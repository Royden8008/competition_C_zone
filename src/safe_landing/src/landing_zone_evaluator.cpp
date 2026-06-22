#include "safe_landing/landing_zone_evaluator.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace safe_landing {

namespace {
constexpr double kDeg = 180.0 / M_PI;
}

LandingZoneEvaluator::LandingZoneEvaluator(ros::NodeHandle& nh,
                                           const std::string& cloud_topic,
                                           const ZoneParams& params)
    : p_(params), cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
  cloud_sub_  = nh.subscribe(cloud_topic, 2, &LandingZoneEvaluator::cloudCb, this);
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "safe_landing/zone_markers", 1, true);
}

bool LandingZoneEvaluator::haveCloud() const {
  std::lock_guard<std::mutex> lk(mu_);
  return !cloud_->empty();
}

void LandingZoneEvaluator::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);  // finer: zone fit needs detail
  vg.filter(*c);
  std::lock_guard<std::mutex> lk(mu_);
  cloud_       = c;
  cloud_stamp_ = msg->header.stamp;
}

ZoneResult LandingZoneEvaluator::evaluatePatch(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    double cx, double cy, double cz) const {
  ZoneResult r;
  r.x = cx; r.y = cy; r.z = cz;

  // 1) Pull points within the disc and a ground-height band around cz.
  pcl::PointCloud<pcl::PointXYZ>::Ptr disc(new pcl::PointCloud<pcl::PointXYZ>);
  disc->reserve(2048);
  const double r2 = p_.disc_radius * p_.disc_radius;
  const double cr2 = p_.clear_radius * p_.clear_radius;
  int clear_pts = 0;
  for (const auto& pt : cloud.points) {
    const double dx = pt.x - cx;
    const double dy = pt.y - cy;
    const double dz = pt.z - cz;
    const double d2 = dx * dx + dy * dy;
    // Clearance cylinder: anything strictly above ground band, within clear_radius.
    if (d2 < cr2 && dz > p_.z_band_lo * 0.5 && dz < p_.clear_height) {
      ++clear_pts;
    }
    if (d2 > r2) continue;
    if (dz < -p_.z_band_lo || dz > p_.z_band_hi) continue;
    disc->push_back(pt);
  }
  r.clearance_violations = clear_pts;
  if (clear_pts > p_.max_clear_pts) {
    r.reason = "clearance: " + std::to_string(clear_pts) + " pts above patch";
    return r;
  }
  if (static_cast<int>(disc->size()) < p_.min_inliers) {
    r.reason = "too few points in disc: " + std::to_string(disc->size());
    return r;
  }

  // 2) RANSAC plane fit.
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(p_.ransac_thresh);
  seg.setMaxIterations(80);
  seg.setInputCloud(disc);
  seg.segment(*inliers, *coeff);

  if (static_cast<int>(inliers->indices.size()) < p_.min_inliers ||
      coeff->values.size() != 4) {
    r.reason = "RANSAC: too few inliers (" +
               std::to_string(inliers->indices.size()) + ")";
    return r;
  }

  // 3) Slope: angle between plane normal and world +Z.
  Eigen::Vector3d n(coeff->values[0], coeff->values[1], coeff->values[2]);
  const double nrm = n.norm();
  if (nrm < 1e-6) { r.reason = "degenerate plane"; return r; }
  n /= nrm;
  if (n.z() < 0) n = -n;
  const double slope = std::acos(std::clamp(n.z(), -1.0, 1.0)) * kDeg;
  r.slope_deg = slope;
  r.inliers   = static_cast<int>(inliers->indices.size());
  if (slope > p_.max_slope_deg) {
    r.reason = "slope " + std::to_string(slope) + " deg";
    return r;
  }

  // 4) Roughness: RMS residual to the fitted plane.
  const double c0 = coeff->values[0], c1 = coeff->values[1], c2 = coeff->values[2];
  const double cn = std::max(1e-9, std::sqrt(c0 * c0 + c1 * c1 + c2 * c2));
  const double a = c0 / cn, b = c1 / cn;
  const double cc = c2 / cn, d = coeff->values[3] / cn;
  double sumsq = 0.0;
  double mean_z = 0.0;
  for (int idx : inliers->indices) {
    const auto& pt = (*disc)[idx];
    const double res = (a * pt.x + b * pt.y + cc * pt.z + d);  // unit normal
    sumsq  += res * res;
    mean_z += pt.z;
  }
  r.roughness = std::sqrt(sumsq / inliers->indices.size());
  r.z         = mean_z / inliers->indices.size();
  if (r.roughness > p_.max_roughness) {
    r.reason = "roughness " + std::to_string(r.roughness) + " m";
    return r;
  }

  r.ok = true;
  return r;
}

ZoneResult LandingZoneEvaluator::evaluate(double tx, double ty, double tz) {
  ZoneResult fail;
  pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (cloud_->empty()) {
      fail.x = tx; fail.y = ty; fail.z = tz;
      fail.reason = "no point cloud yet";
      return fail;
    }
    snapshot = cloud_;
  }

  // Try the requested point first, then a spiral-style grid out to search_radius.
  std::vector<std::pair<double, double>> offsets{{0.0, 0.0}};
  for (double r = p_.search_step; r <= p_.search_radius + 1e-6; r += p_.search_step) {
    const int n = std::max(4, static_cast<int>(std::round(2 * M_PI * r / p_.search_step)));
    for (int i = 0; i < n; ++i) {
      const double th = 2 * M_PI * i / n;
      offsets.emplace_back(r * std::cos(th), r * std::sin(th));
    }
  }

  ZoneResult best = fail;
  best.x = tx; best.y = ty; best.z = tz;
  best.reason = "all candidates failed";
  for (const auto& [dx, dy] : offsets) {
    ZoneResult r = evaluatePatch(*snapshot, tx + dx, ty + dy, tz);
    if (r.ok) {
      publishMarker(r);
      ROS_INFO("landing zone OK at (%.2f, %.2f, %.2f) slope=%.1fdeg rough=%.3fm "
               "inliers=%d clear=%d",
               r.x, r.y, r.z, r.slope_deg, r.roughness,
               r.inliers, r.clearance_violations);
      return r;
    }
    // Track the "least bad" candidate for diagnostics.
    if (r.inliers > best.inliers) best = r;
  }
  publishMarker(best);
  ROS_WARN("landing zone search FAILED: %s", best.reason.c_str());
  return best;
}

void LandingZoneEvaluator::publishMarker(const ZoneResult& r) const {
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = ros::Time::now();
  m.ns   = "safe_landing";
  m.id   = 0;
  m.type = visualization_msgs::Marker::CYLINDER;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = r.x;
  m.pose.position.y = r.y;
  m.pose.position.z = r.z;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = 2.0 * p_.disc_radius;
  m.scale.z = 0.02;
  m.color.a = 0.5;
  m.color.r = r.ok ? 0.1f : 0.9f;
  m.color.g = r.ok ? 0.9f : 0.2f;
  m.color.b = 0.1f;
  arr.markers.push_back(m);
  marker_pub_.publish(arr);
}

}  // namespace safe_landing
