#include "safe_landing/px4_interface.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace safe_landing {

PX4Interface::PX4Interface(ros::NodeHandle& nh) : nh_(nh) {
  state_sub_ = nh_.subscribe("/mavros/state", 10, &PX4Interface::stateCb, this);
  odom_sub_  = nh_.subscribe("/mavros/local_position/odom", 50,
                             &PX4Interface::odomCb, this);
  sp_pub_    = nh_.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 50);

  arming_cli_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mode_cli_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  cmd_cli_    = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

  target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target_.type_mask =
      mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
      mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}

void PX4Interface::stateCb(const mavros_msgs::State::ConstPtr& msg) {
  state_ = *msg;
}

void PX4Interface::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  cx_.store(msg->pose.pose.position.x);
  cy_.store(msg->pose.pose.position.y);
  cz_.store(msg->pose.pose.position.z);
  cvx_.store(msg->twist.twist.linear.x);
  cvy_.store(msg->twist.twist.linear.y);
  cvz_.store(msg->twist.twist.linear.z);

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double r, p, y;
  tf2::Matrix3x3(q).getRPY(r, p, y);
  cyaw_.store(y);
  pose_inited_.store(true);
}

void PX4Interface::setPosition(double x, double y, double z, double yaw) {
  target_.header.stamp = ros::Time::now();
  target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target_.type_mask =
      mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
      mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  target_.position.x = x;
  target_.position.y = y;
  target_.position.z = z;
  target_.yaw = yaw;
}

void PX4Interface::setVelocity(double vx, double vy, double vz, double yaw) {
  target_.header.stamp = ros::Time::now();
  target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target_.type_mask =
      mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
      mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  target_.velocity.x = vx;
  target_.velocity.y = vy;
  target_.velocity.z = vz;
  target_.yaw = yaw;
}

void PX4Interface::holdHere() {
  setPosition(cx_.load(), cy_.load(), cz_.load(), cyaw_.load());
}

void PX4Interface::publish() {
  sp_pub_.publish(target_);
}

bool PX4Interface::setMode(const std::string& mode) {
  mavros_msgs::SetMode req;
  req.request.custom_mode = mode;
  return mode_cli_.call(req) && req.response.mode_sent;
}

bool PX4Interface::arm(bool value) {
  mavros_msgs::CommandBool req;
  req.request.value = value;
  return arming_cli_.call(req) && req.response.success;
}

bool PX4Interface::setServoPwm(int channel, int pwm_us) {
  mavros_msgs::CommandLong req;
  req.request.command = 183;          // MAV_CMD_DO_SET_SERVO
  req.request.param1  = channel;      // 1-based AUX channel
  req.request.param2  = pwm_us;
  return cmd_cli_.call(req) && req.response.success;
}

bool PX4Interface::setOffboardAndArm(double timeout_s) {
  ros::Rate r(20);
  // PX4 needs a stream of setpoints before OFFBOARD is accepted.
  holdHere();
  for (int i = 0; i < 50 && ros::ok(); ++i) {
    publish();
    ros::spinOnce();
    r.sleep();
  }
  ros::Time start = ros::Time::now();
  ros::Time last_mode = ros::Time(0);
  ros::Time last_arm = ros::Time(0);
  while (ros::ok()) {
    publish();
    ros::Time now = ros::Time::now();
    if (state_.mode != "OFFBOARD" && (now - last_mode).toSec() > 1.0) {
      setMode("OFFBOARD");
      last_mode = now;
    }
    if (state_.mode == "OFFBOARD" && !state_.armed && (now - last_arm).toSec() > 1.0) {
      arm(true);
      last_arm = now;
    }
    if (state_.mode == "OFFBOARD" && state_.armed) return true;
    if ((now - start).toSec() > timeout_s) return false;
    ros::spinOnce();
    r.sleep();
  }
  return false;
}

}  // namespace safe_landing
