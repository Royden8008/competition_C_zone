#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace safe_landing {

class PX4Interface;
class FlightLogger;

struct Waypoint {
  // World-frame coordinates (set up so origin = takeoff point + initial yaw).
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;     // AGL
  double yaw = 0.0;   // [rad], relative to initial yaw
  double tol = 0.25;  // [m] arrival tolerance
  double v_max = 0.6; // [m/s] cruise speed cap on this leg
};

struct WaypointParams {
  // Reactive avoidance (forward cone in body frame).
  double cone_half_angle_deg = 25.0;  // forward detection cone
  double cone_radius_min     = 0.25;  // [m] cone radius right at the drone
  double slow_dist           = 1.20;  // [m] start slowing
  double stop_dist           = 0.90;  // [m] hold position
  double emergency_dist      = 0.45;  // [m] brake regardless of planner state
  double braking_margin      = 0.35;  // [m] extra distance beyond v^2/2a
  double abort_hold_s        = 2.0;   // [s] how long stop_dist must persist
  // Side avoidance (sphere around the drone).
  double side_radius         = 0.40;  // [m] radius for omni-directional check
  // Speed shaping.
  double k_pos               = 1.0;   // P-gain on position error → vel cmd
  double accel_limit         = 1.5;   // [m/s^2] vel rate-limit
  double yaw_rate_max        = 1.0;   // [rad/s]
  // Loop.
  double rate_hz             = 30.0;
  double leg_timeout_s       = 30.0;
  // Whether to skip a leg after abort_hold_s, or fail outright.
  bool   skip_blocked_leg    = true;
};

enum class WaypointResult {
  AllReached,    // every leg arrived within tolerance
  PartialSkipped,// at least one leg was skipped due to obstacle
  Aborted,       // gave up (skip disabled and a leg blocked)
  Timeout,
  ModeLost,
};

// Drives the vehicle through a list of world-frame waypoints, each leg
// using a speed-limited position controller. Watches the LiDAR cloud
// in a forward cone and slows / stops / skips when blocked.
class WaypointFollower {
 public:
  WaypointFollower(ros::NodeHandle& nh,
                   PX4Interface& px4,
                   const std::string& cloud_topic,
                   const WaypointParams& params);

  void setWaypoints(const std::vector<Waypoint>& wps);
  void setLogger(FlightLogger* log) { log_ = log; }
  void setCancelCallback(std::function<bool()> cb) { cancel_cb_ = std::move(cb); }
  WaypointResult run();

  // Where the drone ended up (last waypoint actually reached, or current
  // pose if all skipped). Used by the FSM to feed LandingZoneEvaluator.
  double endX() const { return last_x_; }
  double endY() const { return last_y_; }
  double endZ() const { return last_z_; }

 private:
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
  // Min distance to any point inside a forward cone (body frame yaw).
  // Returns +inf if none seen.
  double minForwardObstacle(double yaw_body) const;
  // Min distance to any point within side_radius (excluding ground +Z band).
  double minSideObstacle() const;

  PX4Interface& px4_;
  ros::Subscriber cloud_sub_;
  WaypointParams p_;

  std::vector<Waypoint> wps_;
  double last_x_{0.0}, last_y_{0.0}, last_z_{0.0};
  FlightLogger* log_{nullptr};
  std::function<bool()> cancel_cb_;

  mutable std::mutex mu_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

}  // namespace safe_landing
