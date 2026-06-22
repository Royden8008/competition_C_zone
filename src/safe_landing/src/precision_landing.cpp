#include "safe_landing/precision_landing.h"

#include "safe_landing/px4_interface.h"

#ifdef SAFE_LANDING_HAVE_ALVAR
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#endif

#include <algorithm>
#include <cmath>

namespace safe_landing {

PrecisionLanding::PrecisionLanding(ros::NodeHandle& nh,
                                   PX4Interface& px4,
                                   const PrecisionParams& params)
    : px4_(px4), p_(params) {
#ifdef SAFE_LANDING_HAVE_ALVAR
  if (p_.use_marker) {
    marker_sub_ = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
        p_.marker_topic, 5,
        [this](const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg) {
          if (msg->markers.empty()) return;
          // Pick the configured ID, or the first one.
          const ar_track_alvar_msgs::AlvarMarker* chosen = nullptr;
          for (const auto& m : msg->markers) {
            if (p_.marker_id < 0 || static_cast<int>(m.id) == p_.marker_id) {
              chosen = &m;
              break;
            }
          }
          if (!chosen) return;
          // The marker pose is in the camera frame; we treat its X/Y as
          // the body-frame error (camera pointing down, axes-aligned).
          // If your camera frame differs, remap here.
          std::lock_guard<std::mutex> lk(mu_);
          marker_dx_ = chosen->pose.pose.position.x;
          marker_dy_ = chosen->pose.pose.position.y;
          marker_stamp_ = msg->header.stamp;
          marker_have_ = true;
        });
    ROS_INFO("PrecisionLanding: marker mode on, topic=%s id=%d",
             p_.marker_topic.c_str(), p_.marker_id);
  }
#else
  if (p_.use_marker) {
    ROS_WARN("PrecisionLanding: built without ar_track_alvar_msgs, "
             "marker mode disabled");
    p_.use_marker = false;
  }
#endif
  (void)nh;
}

PrecisionResult PrecisionLanding::run(double ground_z) {
  ros::Rate rate(p_.rate_hz);
  ros::Time start = ros::Time::now();
  const double yaw_hold = px4_.yaw();

  // Phase 1: try to align with the marker (or just hover) until aligned or timeout.
  if (p_.use_marker) {
    while (ros::ok()) {
      const double elapsed = (ros::Time::now() - start).toSec();
      bool have; double dx = 0, dy = 0; ros::Time stamp;
      {
        std::lock_guard<std::mutex> lk(mu_);
        have = marker_have_;
        dx = marker_dx_; dy = marker_dy_;
        stamp = marker_stamp_;
      }
      const bool fresh = have && (ros::Time::now() - stamp).toSec() < 0.5;
      if (fresh && std::hypot(dx, dy) < p_.align_tol_xy) {
        ROS_INFO("precision: aligned with marker (err=%.3fm)", std::hypot(dx, dy));
        break;
      }
      if (elapsed > p_.align_timeout_s) {
        ROS_WARN("precision: marker align timeout, continuing without alignment");
        break;
      }

      double vx = 0.0, vy = 0.0;
      if (fresh) {
        // Move toward the tag: tag's body-X means drone should move +X.
        vx = std::clamp(p_.kp_xy * dx, -p_.max_v_xy, p_.max_v_xy);
        vy = std::clamp(p_.kp_xy * dy, -p_.max_v_xy, p_.max_v_xy);
      }
      // Hover Z while aligning.
      px4_.setVelocity(vx, vy, 0.0, yaw_hold);
      px4_.publish();
      ros::spinOnce();
      rate.sleep();
    }
  }

  // Phase 2: hand off to PX4 AUTO.LAND. PX4's land controller handles
  // touchdown detection and disarm.
  ROS_INFO("precision: switching to AUTO.LAND");
  for (int i = 0; i < 5; ++i) {
    if (px4_.setMode("AUTO.LAND")) break;
    ros::Duration(0.2).sleep();
  }

  // Wait for PX4 to disarm (touchdown) or for an outer timeout.
  ros::Time t0 = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - t0).toSec() < 15.0) {
    if (!px4_.armed()) {
      ROS_INFO("precision: vehicle disarmed, landing complete");
      return PrecisionResult::Landed;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return PrecisionResult::Handed;
}

}  // namespace safe_landing
