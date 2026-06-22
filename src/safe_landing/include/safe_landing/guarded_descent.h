#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <mutex>

namespace safe_landing {

class PX4Interface;
class FlightLogger;

struct DescentParams {
  double vz_nominal     = -0.30;  // [m/s] cruise descent speed (negative = down)
  double vz_slow        = -0.15;  // [m/s] near obstacle
  double slow_dist      =  0.40;  // [m]   start slowing
  double stop_dist      =  0.20;  // [m]   hover instead of descending
  double abort_dist     =  0.15;  // [m]   trigger abort if held this close
  double abort_hold_s   =  2.0;   // [s]   how long stop_dist must persist
  double cone_half_angle_deg = 20.0; // detection cone below the drone
  double cone_radius    =  0.30;  // [m]   horizontal radius at the drone
  double target_z       =  0.30;  // [m]   handoff altitude (AGL) to AUTO.LAND
  double rate_hz        = 30.0;   // control-loop rate
};

enum class DescentResult {
  Reached,    // got down to target_z without trouble
  Aborted,    // obstacle too close for too long
  Timeout,    // took longer than budget
  ModeLost,   // OFFBOARD dropped or vehicle disarmed
};

// Drives the vehicle down at a velocity setpoint while watching the
// LiDAR cloud directly below it. Slows / hovers / aborts based on the
// nearest point inside a cone under the drone.
class GuardedDescent {
 public:
  GuardedDescent(ros::NodeHandle& nh,
                 PX4Interface& px4,
                 const std::string& cloud_topic,
                 const DescentParams& params);

  // Blocks until one of DescentResult is reached. ground_z is the world-Z
  // of the ground at the chosen landing point (from LandingZoneEvaluator).
  DescentResult run(double ground_z, double timeout_s = 30.0);
  void setLogger(FlightLogger* log) { log_ = log; }

 private:
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
  // Min distance (along -Z) to any point inside the detection cone.
  // Returns +inf if no points are seen.
  double minObstacleDistance() const;

  PX4Interface& px4_;
  ros::Subscriber cloud_sub_;
  DescentParams p_;

  mutable std::mutex mu_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  FlightLogger* log_{nullptr};
};

}  // namespace safe_landing
