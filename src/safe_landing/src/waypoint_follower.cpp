#include "safe_landing/waypoint_follower.h"

#include "safe_landing/flight_logger.h"
#include "safe_landing/px4_interface.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace safe_landing {

namespace {
constexpr double kEps = 1e-6;
}

WaypointFollower::WaypointFollower(ros::NodeHandle& nh,
                                   PX4Interface& px4,
                                   const std::string& cloud_topic,
                                   const WaypointParams& params)
    : px4_(px4), p_(params), cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
  cloud_sub_ = nh.subscribe(cloud_topic, 2, &WaypointFollower::cloudCb, this);
}

void WaypointFollower::setWaypoints(const std::vector<Waypoint>& wps) {
  wps_ = wps;
}

void WaypointFollower::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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

double WaypointFollower::minForwardObstacle(double yaw_body) const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr snap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = cloud_;
  }
  if (!snap || snap->empty()) return std::numeric_limits<double>::infinity();

  const double dx0 = px4_.x();
  const double dy0 = px4_.y();
  const double dz0 = px4_.z();
  const double cy = std::cos(yaw_body);
  const double sy = std::sin(yaw_body);
  const double tan_h = std::tan(p_.cone_half_angle_deg * M_PI / 180.0);

  double best = std::numeric_limits<double>::infinity();
  for (const auto& pt : snap->points) {
    const double rx = pt.x - dx0;
    const double ry = pt.y - dy0;
    const double rz = pt.z - dz0;
    // Project into body frame: forward = +x_body.
    const double xf =  cy * rx + sy * ry;
    const double yf = -sy * rx + cy * ry;
    if (xf < 0.10) continue;            // ignore points behind / right at us
    // Cone allows lateral / vertical offset growing with depth.
    const double rmax = p_.cone_radius_min + xf * tan_h;
    if (yf * yf + rz * rz > rmax * rmax) continue;
    if (xf < best) best = xf;
  }
  return best;
}

double WaypointFollower::minSideObstacle() const {
  pcl::PointCloud<pcl::PointXYZ>::Ptr snap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = cloud_;
  }
  if (!snap || snap->empty()) return std::numeric_limits<double>::infinity();

  const double dx0 = px4_.x();
  const double dy0 = px4_.y();
  const double dz0 = px4_.z();
  const double r2_max = p_.side_radius * p_.side_radius;
  // Exclude ground returns: ignore points more than 0.2 m below us.
  double best = std::numeric_limits<double>::infinity();
  for (const auto& pt : snap->points) {
    const double rx = pt.x - dx0;
    const double ry = pt.y - dy0;
    const double rz = pt.z - dz0;
    if (rz < -0.20) continue;
    const double d2 = rx * rx + ry * ry + rz * rz;
    if (d2 > r2_max) continue;
    const double d = std::sqrt(d2);
    if (d < best) best = d;
  }
  return best;
}

WaypointResult WaypointFollower::run() {
  if (wps_.empty()) return WaypointResult::AllReached;

  // Seed "where we ended up" with the current pose, so a fully-skipped run
  // reports a real position instead of (0,0).
  last_x_ = px4_.x(); last_y_ = px4_.y(); last_z_ = px4_.z();

  ros::Rate rate(p_.rate_hz);
  bool any_skipped = false;

  // Velocity rate-limit memory.
  double vx_cmd = 0.0, vy_cmd = 0.0, vz_cmd = 0.0;
  const double dt = 1.0 / p_.rate_hz;
  const double dv_max = p_.accel_limit * dt;

  for (size_t i = 0; i < wps_.size(); ++i) {
    if (cancel_cb_ && cancel_cb_()) {
      ROS_WARN("waypoint: canceled before leg %zu", i + 1);
      px4_.holdHere();
      px4_.publish();
      return WaypointResult::Aborted;
    }
    const Waypoint& w = wps_[i];
    ros::Time leg_start = ros::Time::now();
    ros::Time first_blocked;
    bool blocked_active = false;

    // Diagnostics. The "planned line" for this leg runs from the previous
    // waypoint (or current pose for the first leg) to the target; cross-track
    // is the perpendicular distance from it — the direct measure of corner
    // cutting. min_d_side / min_d_fwd capture the closest approach we allowed.
    const double leg_ax = (i > 0) ? wps_[i - 1].x : px4_.x();
    const double leg_ay = (i > 0) ? wps_[i - 1].y : px4_.y();
    double max_cross  = 0.0;
    double min_d_side = std::numeric_limits<double>::infinity();
    double min_d_fwd  = std::numeric_limits<double>::infinity();

    ROS_INFO("[wp %zu/%zu] target=(%.2f, %.2f, %.2f) yaw=%.2f tol=%.2f vmax=%.2f",
             i + 1, wps_.size(), w.x, w.y, w.z, w.yaw, w.tol, w.v_max);

    while (ros::ok()) {
      if (cancel_cb_ && cancel_cb_()) {
        ROS_WARN("waypoint: canceled by external controller");
        px4_.holdHere();
        px4_.publish();
        return WaypointResult::Aborted;
      }
      if (px4_.mode() != "OFFBOARD" || !px4_.armed()) {
        ROS_ERROR("waypoint: lost OFFBOARD/armed (mode=%s armed=%d)",
                  px4_.mode().c_str(), px4_.armed());
        return WaypointResult::ModeLost;
      }

      // ---- position error ----
      const double ex = w.x - px4_.x();
      const double ey = w.y - px4_.y();
      const double ez = w.z - px4_.z();
      const double d_xy = std::hypot(ex, ey);
      const double d_3d = std::sqrt(ex * ex + ey * ey + ez * ez);
      if (d_3d < w.tol) {
        ROS_INFO("[wp %zu] reached (d=%.2f) | leg max_cross=%.2f min_d_fwd=%.2f min_d_side=%.2f",
                 i + 1, d_3d, max_cross, min_d_fwd, min_d_side);
        last_x_ = px4_.x(); last_y_ = px4_.y(); last_z_ = px4_.z();
        break;
      }

      // ---- desired velocity (P, capped at v_max) ----
      double vx_des = p_.k_pos * ex;
      double vy_des = p_.k_pos * ey;
      double vz_des = p_.k_pos * ez;
      const double v_xy = std::hypot(vx_des, vy_des);
      if (v_xy > w.v_max) {
        const double s = w.v_max / v_xy;
        vx_des *= s;
        vy_des *= s;
      }
      vz_des = std::clamp(vz_des, -0.5, 0.5);

      // ---- yaw: face the direction of motion if leg is long enough ----
      double yaw_cmd = w.yaw;
      if (d_xy > 0.5) {
        yaw_cmd = std::atan2(ey, ex);
      }

      // ---- reactive avoidance ----
      // Forward cone uses commanded heading (not yaw_cmd which may face goal),
      // because the body actually moves along v_des.
      const double v_dir = std::atan2(vy_des, vx_des);
      const double d_fwd  = (v_xy > 0.05)
                              ? minForwardObstacle(v_dir)
                              : std::numeric_limits<double>::infinity();
      const double d_side = minSideObstacle();
      const double stop_dist = std::max(p_.stop_dist,
                                        px4_.speedXY() * px4_.speedXY() /
                                            (2.0 * std::max(p_.accel_limit, kEps)) +
                                        p_.braking_margin);
      double scale = 1.0;
      if (d_fwd < p_.emergency_dist || d_side < p_.side_radius) {
        scale = 0.0;
        vx_cmd = 0.0;
        vy_cmd = 0.0;
        vz_cmd = 0.0;
      } else if (d_fwd < stop_dist) {
        scale = 0.0;
      } else if (d_fwd < p_.slow_dist) {
        scale = (d_fwd - stop_dist) / (p_.slow_dist - stop_dist + kEps);
        scale = std::clamp(scale, 0.0, 1.0);
      }
      vx_des *= scale;
      vy_des *= scale;

      // ---- diagnostics: cross-track from planned line + closest approach ----
      const double abx = w.x - leg_ax, aby = w.y - leg_ay;
      const double seg_len = std::hypot(abx, aby);
      double cross = 0.0;
      if (seg_len > 1e-3) {
        cross = std::abs(abx * (px4_.y() - leg_ay) - aby * (px4_.x() - leg_ax)) / seg_len;
      }
      max_cross = std::max(max_cross, cross);
      if (d_side < min_d_side) min_d_side = d_side;
      if (std::isfinite(d_fwd) && d_fwd < min_d_fwd) min_d_fwd = d_fwd;
      ROS_INFO_THROTTLE(0.5,
          "[wp %zu] pos=(%.2f,%.2f) cross=%.2f d_fwd=%.2f d_side=%.2f scale=%.2f v=%.2f",
          i + 1, px4_.x(), px4_.y(), cross, d_fwd, d_side, scale, px4_.speedXY());

      // ---- block detection ----
      if (scale < 0.05) {
        if (!blocked_active) {
          blocked_active = true;
          first_blocked = ros::Time::now();
          ROS_WARN_THROTTLE(1, "[wp %zu] blocked (d_fwd=%.2f d_side=%.2f stop=%.2f)",
                            i + 1, d_fwd, d_side, stop_dist);
          if (log_) log_->event("Cruise", px4_, "blocked");
        } else if ((ros::Time::now() - first_blocked).toSec() > p_.abort_hold_s) {
          if (p_.skip_blocked_leg) {
            ROS_WARN("[wp %zu] blocked >%.1fs, SKIP -> replan | max_cross=%.2f min_d_fwd=%.2f min_d_side=%.2f",
                     i + 1, p_.abort_hold_s, max_cross, min_d_fwd, min_d_side);
            if (log_) log_->event("Cruise", px4_, "skip_leg");
            any_skipped = true;
            last_x_ = px4_.x(); last_y_ = px4_.y(); last_z_ = px4_.z();
            // Bail the whole segment, not just this leg: the outer loop will
            // replan from this (stopped, safe) pose. Continuing to the next
            // leg would march deeper into the same obstacle.
            px4_.holdHere();
            for (int k = 0; k < 10; ++k) { px4_.publish(); rate.sleep(); }
            return WaypointResult::PartialSkipped;
          } else {
            ROS_ERROR("[wp %zu] blocked, abort", i + 1);
            return WaypointResult::Aborted;
          }
        }
      } else {
        blocked_active = false;
      }

      // ---- accel-limit ----
      vx_cmd += std::clamp(vx_des - vx_cmd, -dv_max, dv_max);
      vy_cmd += std::clamp(vy_des - vy_cmd, -dv_max, dv_max);
      vz_cmd += std::clamp(vz_des - vz_cmd, -dv_max, dv_max);

      px4_.setVelocity(vx_cmd, vy_cmd, vz_cmd, yaw_cmd);
      px4_.publish();
      if (log_) log_->sampleCruise("Cruise", px4_, d_fwd, d_side, scale, w.x, w.y);

      if ((ros::Time::now() - leg_start).toSec() > p_.leg_timeout_s) {
        ROS_WARN("[wp %zu] timeout (%.1fs), moving on | max_cross=%.2f min_d_fwd=%.2f min_d_side=%.2f",
                 i + 1, p_.leg_timeout_s, max_cross, min_d_fwd, min_d_side);
        if (log_) log_->event("Cruise", px4_, "leg_timeout");
        any_skipped = true;
        last_x_ = px4_.x(); last_y_ = px4_.y(); last_z_ = px4_.z();
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

  return any_skipped ? WaypointResult::PartialSkipped : WaypointResult::AllReached;
}

}  // namespace safe_landing
