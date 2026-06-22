#include "safe_landing/guarded_descent.h"

#include "safe_landing/flight_logger.h"
#include "safe_landing/px4_interface.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace safe_landing {

GuardedDescent::GuardedDescent(ros::NodeHandle& nh,
                               PX4Interface& px4,
                               const std::string& cloud_topic,
                               const DescentParams& params)
    : px4_(px4), p_(params), cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
  cloud_sub_ = nh.subscribe(cloud_topic, 2, &GuardedDescent::cloudCb, this);
}

void GuardedDescent::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(0.10f, 0.10f, 0.10f);
  vg.filter(*c);
  std::lock_guard<std::mutex> lk(mu_);
  cloud_ = c;
}

double GuardedDescent::minObstacleDistance() const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr snap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = cloud_;
  }
  if (!snap || snap->empty()) return std::numeric_limits<double>::infinity();

  const double dx0 = px4_.x();
  const double dy0 = px4_.y();
  const double dz0 = px4_.z();
  const double tan_h = std::tan(p_.cone_half_angle_deg * M_PI / 180.0);

  double best = std::numeric_limits<double>::infinity();
  for (const auto& pt : snap->points) {
    const double dx = pt.x - dx0;
    const double dy = pt.y - dy0;
    const double dz = dz0 - pt.z;        // positive when point is below us
    const double r2 = dx * dx + dy * dy;
    // Side/overhang guard: any point roughly at our height within cone_radius
    // (e.g. railings, branches) counts as an obstacle at its horizontal range.
    if (std::abs(dz) < 0.30 && r2 < p_.cone_radius * p_.cone_radius) {
      const double hr = std::sqrt(r2);
      if (hr < best) best = hr;
      continue;
    }
    if (dz <= 0.05) continue;             // ignore points at/above us
    // Cone widens with depth: allowed radius = cone_radius + dz * tan(angle).
    const double rmax = p_.cone_radius + dz * tan_h;
    if (r2 > rmax * rmax) continue;
    if (dz < best) best = dz;
  }
  return best;
}

DescentResult GuardedDescent::run(double ground_z, double timeout_s) {
  const double yaw_hold = px4_.yaw();
  ros::Rate rate(p_.rate_hz);
  ros::Time start = ros::Time::now();
  ros::Time first_too_close;
  bool too_close_active = false;

  while (ros::ok()) {
    if (px4_.mode() != "OFFBOARD" || !px4_.armed()) {
      ROS_ERROR("descent: lost OFFBOARD/armed (mode=%s armed=%d)",
                px4_.mode().c_str(), px4_.armed());
      return DescentResult::ModeLost;
    }
    const double agl = px4_.z() - ground_z;
    if (agl <= p_.target_z) {
      px4_.holdHere();
      px4_.publish();
      ROS_INFO("descent: reached target AGL %.2f m", agl);
      return DescentResult::Reached;
    }

    const double d = minObstacleDistance();
    double vz = p_.vz_nominal;
    if (d < p_.stop_dist) {
      vz = 0.0;                         // hover
      if (!too_close_active) {
        too_close_active = true;
        first_too_close = ros::Time::now();
      } else if ((ros::Time::now() - first_too_close).toSec() > p_.abort_hold_s &&
                 d < p_.abort_dist) {
        ROS_WARN("descent: ABORT — obstacle at %.2fm for >%.1fs",
                 d, p_.abort_hold_s);
        if (log_) log_->event("Descend", px4_, "abort");
        px4_.holdHere();
        px4_.publish();
        return DescentResult::Aborted;
      }
    } else if (d < p_.slow_dist) {
      vz = p_.vz_slow;
      too_close_active = false;
    } else {
      too_close_active = false;
    }

    px4_.setVelocity(0.0, 0.0, vz, yaw_hold);
    px4_.publish();
    if (log_) log_->sampleDescent("Descend", px4_, d);

    if ((ros::Time::now() - start).toSec() > timeout_s) {
      ROS_WARN("descent: timeout (%.1fs)", timeout_s);
      return DescentResult::Timeout;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return DescentResult::Timeout;
}

}  // namespace safe_landing
