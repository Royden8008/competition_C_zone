#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <string>

namespace safe_landing {

// Thin MAVROS wrapper. Holds the latest odometry/state and a single
// PositionTarget that callers fill in different modes (position / velocity).
// All read accessors are lock-free atomics so the FSM thread can poll cheaply.
class PX4Interface {
 public:
  explicit PX4Interface(ros::NodeHandle& nh);

  // ---- live state ----
  bool   connected() const { return state_.connected; }
  bool   armed()     const { return state_.armed; }
  std::string mode() const { return state_.mode; }
  bool   poseReady() const { return pose_inited_.load(); }

  double x()   const { return cx_.load(); }
  double y()   const { return cy_.load(); }
  double z()   const { return cz_.load(); }
  double yaw() const { return cyaw_.load(); }
  double vx()  const { return cvx_.load(); }
  double vy()  const { return cvy_.load(); }
  double vz()  const { return cvz_.load(); }
  double speedXY() const { return std::hypot(vx(), vy()); }

  // ---- setpoint helpers (fill the internal target; publish() sends it) ----
  void setPosition(double x, double y, double z, double yaw);
  void setVelocity(double vx, double vy, double vz, double yaw);
  // Hover at current pose (used as a safe default during transitions).
  void holdHere();
  // Push the current target to /mavros/setpoint_raw/local. Call at >= 20 Hz.
  void publish();

  // ---- mode / arming / pwm ----
  bool setOffboardAndArm(double timeout_s = 10.0);
  bool setMode(const std::string& mode);
  bool arm(bool value);
  // PX4 AUX channels via DO_SET_SERVO. Channels are 1-based per MAV_CMD spec.
  bool setServoPwm(int channel, int pwm_us);

 private:
  void stateCb(const mavros_msgs::State::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_, odom_sub_;
  ros::Publisher  sp_pub_;
  ros::ServiceClient arming_cli_, mode_cli_, cmd_cli_;

  mavros_msgs::State state_;
  mavros_msgs::PositionTarget target_;

  std::atomic<bool>   pose_inited_{false};
  std::atomic<double> cx_{0.0}, cy_{0.0}, cz_{0.0}, cyaw_{0.0};
  std::atomic<double> cvx_{0.0}, cvy_{0.0}, cvz_{0.0};
};

}  // namespace safe_landing
