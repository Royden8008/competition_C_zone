#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <atomic>
#include <mutex>
#include <string>

namespace safe_landing {

class PX4Interface;

struct PrecisionParams {
  bool   use_marker        = false;   // enable AR-tag fine alignment
  std::string marker_topic = "/ar_pose_marker";
  int    marker_id         = -1;      // -1 = accept first marker seen
  double align_tol_xy      = 0.05;    // [m] horizontal tolerance to be "aligned"
  double align_timeout_s   = 5.0;     // [s] give up on marker, land anyway
  double kp_xy             = 0.6;     // P gain for marker-based correction
  double max_v_xy          = 0.20;    // [m/s] cap on correction velocity
  double handoff_z_agl     = 0.30;    // [m] altitude at which we switch to AUTO.LAND
  double rate_hz           = 30.0;
};

enum class PrecisionResult {
  Landed,        // PX4 reports disarmed after AUTO.LAND
  Handed,        // AUTO.LAND command sent (still descending)
  Timeout,
};

// Optional last-mile alignment using an AR tag, then hand off to PX4 AUTO.LAND.
// Works without a marker too (just descends straight then hands off).
class PrecisionLanding {
 public:
  PrecisionLanding(ros::NodeHandle& nh,
                   PX4Interface& px4,
                   const PrecisionParams& params);

  // ground_z is the world-Z of the ground (used to compute AGL).
  PrecisionResult run(double ground_z);

 private:
#ifdef SAFE_LANDING_HAVE_ALVAR
  void markerCb(const ros::MessageEvent<void const>& evt);
#endif

  PX4Interface& px4_;
  ros::Subscriber marker_sub_;
  PrecisionParams p_;

  std::mutex mu_;
  bool   marker_have_{false};
  double marker_dx_{0.0};   // tag position in body frame, x = forward
  double marker_dy_{0.0};
  ros::Time marker_stamp_;
};

}  // namespace safe_landing
