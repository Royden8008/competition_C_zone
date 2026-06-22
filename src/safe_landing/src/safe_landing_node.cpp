#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "safe_landing/grid_planner.h"
#include "safe_landing/guarded_descent.h"
#include "safe_landing/flight_logger.h"
#include "safe_landing/landing_zone_evaluator.h"
#include "safe_landing/precision_landing.h"
#include "safe_landing/px4_interface.h"
#include "safe_landing/waypoint_follower.h"

#include <cmath>
#include <functional>
#include <vector>

using namespace safe_landing;

using CancelFn = std::function<bool()>;

// Hover in place while keeping the setpoint stream alive for `secs`.
// PX4 drops OFFBOARD if setpoints stop for ~0.5s, so never use a bare sleep
// while airborne.
static void holdFor(PX4Interface& px4, double secs) {
  ros::Rate r(30.0);
  const ros::Time t0 = ros::Time::now();
  while (ros::ok() && (ros::Time::now() - t0).toSec() < secs) {
    px4.holdHere();
    px4.publish();
    ros::spinOnce();
    r.sleep();
  }
}

static bool planWithRetries(GridPlanner& planner,
                            PX4Interface& px4,
                            const PlannerParams& params,
                            double ground_z,
                            const Waypoint& goal,
                            std::vector<Waypoint>& segment,
                            const CancelFn& canceled = CancelFn()) {
  segment.clear();
  if (canceled && canceled()) return false;
  // Fallbacks only widen the search box (allow longer detours); they never
  // shrink inflation below the reactive stop_dist, or A* would emit paths the
  // follower rejects (-> blocked/SKIP storm). Both radii are kept >= stop_dist
  // via config.
  const std::vector<std::pair<double, double>> attempts = {
      {params.inflation_radius, params.xy_margin},
      {params.relaxed_inflation_radius, params.relaxed_xy_margin},
      {params.relaxed_inflation_radius, params.relaxed_xy_margin * 1.6},
      {params.relaxed_inflation_radius, params.relaxed_xy_margin * 2.2},
  };

  for (size_t ai = 0; ai < attempts.size(); ++ai) {
    if (canceled && canceled()) return false;
    const auto& attempt = attempts[ai];
    if (planner.plan(px4.x(), px4.y(), ground_z, goal,
                     attempt.first, attempt.second, segment) && !segment.empty()) {
      // Fell back past the nominal attempt -> thinner margin, higher graze risk.
      // Surface it so a graze can be correlated with the relaxed inflation.
      if (ai > 0) {
        ROS_WARN("planner: used relaxed attempt #%zu (inflation=%.2f margin=%.2f) "
                 "— margin is tighter, watch clearance",
                 ai, attempt.first, attempt.second);
      }
      return true;
    }
  }
  return false;
}

static bool scanAndPlan(GridPlanner& planner,
                        PX4Interface& px4,
                        const PlannerParams& params,
                        double ground_z,
                        const Waypoint& goal,
                        std::vector<Waypoint>& segment,
                        const CancelFn& canceled = CancelFn()) {
  if (canceled && canceled()) return false;
  if (planWithRetries(planner, px4, params, ground_z, goal, segment, canceled)) return true;

  ros::Rate rate(30.0);
  const double yaw0 = px4.yaw();
  const double step = std::max(5.0, params.scan_yaw_step_deg) * M_PI / 180.0;
  const double max_yaw = std::max(0.0, params.scan_max_yaw_deg) * M_PI / 180.0;
  const std::vector<double> signs = {1.0, -1.0};

  for (double delta = step; delta <= max_yaw + 1e-6 && ros::ok(); delta += step) {
    for (double sign : signs) {
      if (canceled && canceled()) {
        px4.holdHere();
        px4.publish();
        return false;
      }
      const double yaw_cmd = yaw0 + sign * delta;
      const ros::Time t0 = ros::Time::now();
      while (ros::ok() && (ros::Time::now() - t0).toSec() < params.scan_settle_s) {
        if (canceled && canceled()) {
          px4.holdHere();
          px4.publish();
          return false;
        }
        px4.setPosition(px4.x(), px4.y(), px4.z(), yaw_cmd);
        px4.publish();
        ros::spinOnce();
        rate.sleep();
      }
      if (planWithRetries(planner, px4, params, ground_z, goal, segment, canceled)) {
        ROS_WARN("planner: found path after yaw scan %.0f deg", sign * delta * 180.0 / M_PI);
        return true;
      }
    }
  }
  px4.setPosition(px4.x(), px4.y(), px4.z(), yaw0);
  return false;
}

static WaypointResult runRollingCruise(const std::vector<Waypoint>& goals,
                                       GridPlanner& planner,
                                       WaypointFollower& wpf,
                                       PX4Interface& px4,
                                       const PlannerParams& params,
                                       double ground_z,
                                       FlightLogger* log,
                                       const CancelFn& canceled = CancelFn()) {
  if (goals.empty()) return WaypointResult::AllReached;

  bool any_skipped = false;
  for (size_t goal_i = 0; goal_i < goals.size(); ++goal_i) {
    const Waypoint& goal = goals[goal_i];
    int replans = 0;  // per-goal budget, so later goals aren't starved
    while (ros::ok()) {
      if (canceled && canceled()) {
        ROS_WARN("planner: canceled by external controller");
        px4.holdHere();
        px4.publish();
        return WaypointResult::Aborted;
      }
      const double d_goal = std::hypot(goal.x - px4.x(), goal.y - px4.y());
      if (d_goal < goal.tol) break;
      if (replans++ >= params.max_replans) {
        ROS_ERROR("planner: exceeded max_replans=%d on goal %zu",
                  params.max_replans, goal_i + 1);
        return WaypointResult::Timeout;
      }
      if (log) log->event("Cruise", px4, "replan");

      std::vector<Waypoint> segment;
      bool ok = scanAndPlan(planner, px4, params, ground_z, goal, segment, canceled);
      if (canceled && canceled()) {
        ROS_WARN("planner: canceled while planning");
        px4.holdHere();
        px4.publish();
        return WaypointResult::Aborted;
      }
      if (!ok || segment.empty()) {
        ROS_ERROR("planner: failed from current pose to goal %zu, holding position", goal_i + 1);
        if (log) log->event("Cruise", px4, "plan_failed");
        px4.holdHere();
        ros::Rate hold_rate(30.0);
        const ros::Time hold_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - hold_start).toSec() < 1.0) {
          px4.publish();
          ros::spinOnce();
          hold_rate.sleep();
        }
        return WaypointResult::Aborted;
      }

      const size_t n = std::min(segment.size(),
                                static_cast<size_t>(std::max(1, params.execute_points)));
      std::vector<Waypoint> short_segment(segment.begin(), segment.begin() + n);
      wpf.setWaypoints(short_segment);
      const WaypointResult wr = wpf.run();
      if (wr == WaypointResult::PartialSkipped) any_skipped = true;
      if (wr != WaypointResult::AllReached && wr != WaypointResult::PartialSkipped) return wr;
    }
  }
  return any_skipped ? WaypointResult::PartialSkipped : WaypointResult::AllReached;
}

struct AppParams {
  // topics
  std::string cloud_topic        = "/cloud_registered";
  std::string marker_topic       = "/ar_pose_marker";
  // flight log
  std::string log_dir            = "/tmp/safe_landing_logs";
  // mission
  bool   enabled                 = true;
  bool   external_control        = false;
  bool   cruise_only             = false;
  double cruise_height           = 1.00;
  double takeoff_timeout         = 15.0;
  double cloud_wait_timeout      = 8.0;
  // landing target (world frame). If left at NaN we use the final waypoint.
  double land_x                  = std::nan("");
  double land_y                  = std::nan("");
  // marker
  bool   use_marker              = false;
  int    marker_id               = -1;
  // sub-module params
  WaypointParams   waypoint;
  PlannerParams    planner;
  ZoneParams       zone;
  DescentParams    descent;
  PrecisionParams  precision;
  // raw waypoint list
  std::vector<Waypoint> waypoints;
};

static bool xmlAsDouble(XmlRpc::XmlRpcValue& v, double& out) {
  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) { out = static_cast<double>(v); return true; }
  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)    { out = static_cast<int>(v);    return true; }
  return false;
}

static void loadWaypoints(ros::NodeHandle& pnh, std::vector<Waypoint>& out) {
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam("waypoints", list)) {
    ROS_WARN("safe_landing: no `waypoints` param, mission will only do takeoff+land");
    return;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("safe_landing: `waypoints` must be a list of dicts");
    return;
  }
  for (int i = 0; i < list.size(); ++i) {
    XmlRpc::XmlRpcValue& item = list[i];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
    Waypoint w;
    xmlAsDouble(item["x"],     w.x);
    xmlAsDouble(item["y"],     w.y);
    xmlAsDouble(item["z"],     w.z);
    if (item.hasMember("yaw"))   xmlAsDouble(item["yaw"],   w.yaw);
    if (item.hasMember("tol"))   xmlAsDouble(item["tol"],   w.tol);
    if (item.hasMember("v_max")) xmlAsDouble(item["v_max"], w.v_max);
    out.push_back(w);
  }
  ROS_INFO("safe_landing: loaded %zu waypoints", out.size());
}

static void loadParams(ros::NodeHandle& pnh, AppParams& a) {
  pnh.param("cloud_topic",     a.cloud_topic,     a.cloud_topic);
  pnh.param("marker_topic",    a.marker_topic,    a.marker_topic);
  pnh.param("log_dir",         a.log_dir,         a.log_dir);
  pnh.param("enabled",         a.enabled,         a.enabled);
  pnh.param("external_control", a.external_control, a.external_control);
  pnh.param("cruise_only",     a.cruise_only,     a.cruise_only);
  pnh.param("cruise_height",   a.cruise_height,   a.cruise_height);
  pnh.param("takeoff_timeout", a.takeoff_timeout, a.takeoff_timeout);
  pnh.param("cloud_wait_timeout", a.cloud_wait_timeout, a.cloud_wait_timeout);
  pnh.param("land_x",          a.land_x,          a.land_x);
  pnh.param("land_y",          a.land_y,          a.land_y);
  pnh.param("use_marker",      a.use_marker,      a.use_marker);
  pnh.param("marker_id",       a.marker_id,       a.marker_id);

  pnh.param("waypoint/cone_half_angle_deg", a.waypoint.cone_half_angle_deg, a.waypoint.cone_half_angle_deg);
  pnh.param("waypoint/cone_radius_min",     a.waypoint.cone_radius_min,     a.waypoint.cone_radius_min);
  pnh.param("waypoint/slow_dist",           a.waypoint.slow_dist,           a.waypoint.slow_dist);
  pnh.param("waypoint/stop_dist",           a.waypoint.stop_dist,           a.waypoint.stop_dist);
  pnh.param("waypoint/emergency_dist",      a.waypoint.emergency_dist,      a.waypoint.emergency_dist);
  pnh.param("waypoint/braking_margin",      a.waypoint.braking_margin,      a.waypoint.braking_margin);
  pnh.param("waypoint/abort_hold_s",        a.waypoint.abort_hold_s,        a.waypoint.abort_hold_s);
  pnh.param("waypoint/side_radius",         a.waypoint.side_radius,         a.waypoint.side_radius);
  pnh.param("waypoint/k_pos",               a.waypoint.k_pos,               a.waypoint.k_pos);
  pnh.param("waypoint/accel_limit",         a.waypoint.accel_limit,         a.waypoint.accel_limit);
  pnh.param("waypoint/leg_timeout_s",       a.waypoint.leg_timeout_s,       a.waypoint.leg_timeout_s);
  pnh.param("waypoint/skip_blocked_leg",    a.waypoint.skip_blocked_leg,    a.waypoint.skip_blocked_leg);

  pnh.param("planner/enabled",            a.planner.enabled,            a.planner.enabled);
  pnh.param("planner/resolution",         a.planner.resolution,         a.planner.resolution);
  pnh.param("planner/inflation_radius",   a.planner.inflation_radius,   a.planner.inflation_radius);
  pnh.param("planner/xy_margin",          a.planner.xy_margin,          a.planner.xy_margin);
  pnh.param("planner/obstacle_min_z_agl", a.planner.obstacle_min_z_agl, a.planner.obstacle_min_z_agl);
  pnh.param("planner/obstacle_max_z_agl", a.planner.obstacle_max_z_agl, a.planner.obstacle_max_z_agl);
  pnh.param("planner/waypoint_spacing",   a.planner.waypoint_spacing,   a.planner.waypoint_spacing);
  pnh.param("planner/waypoint_tol",       a.planner.waypoint_tol,       a.planner.waypoint_tol);
  pnh.param("planner/waypoint_v_max",     a.planner.waypoint_v_max,     a.planner.waypoint_v_max);
  pnh.param("planner/max_plan_points",    a.planner.max_plan_points,    a.planner.max_plan_points);
  pnh.param("planner/execute_points",     a.planner.execute_points,     a.planner.execute_points);
  pnh.param("planner/max_replans",        a.planner.max_replans,        a.planner.max_replans);
  pnh.param("planner/scan_yaw_step_deg",  a.planner.scan_yaw_step_deg,  a.planner.scan_yaw_step_deg);
  pnh.param("planner/scan_max_yaw_deg",   a.planner.scan_max_yaw_deg,   a.planner.scan_max_yaw_deg);
  pnh.param("planner/scan_settle_s",      a.planner.scan_settle_s,      a.planner.scan_settle_s);
  pnh.param("planner/relaxed_inflation_radius", a.planner.relaxed_inflation_radius, a.planner.relaxed_inflation_radius);
  pnh.param("planner/relaxed_xy_margin",        a.planner.relaxed_xy_margin,        a.planner.relaxed_xy_margin);
  pnh.param("planner/cluster_filter_enabled",  a.planner.cluster_filter_enabled,  a.planner.cluster_filter_enabled);
  pnh.param("planner/cluster_tolerance",       a.planner.cluster_tolerance,       a.planner.cluster_tolerance);
  pnh.param("planner/cluster_min_points",      a.planner.cluster_min_points,      a.planner.cluster_min_points);
  pnh.param("planner/cluster_max_points",      a.planner.cluster_max_points,      a.planner.cluster_max_points);

  pnh.param("zone/disc_radius",     a.zone.disc_radius,     a.zone.disc_radius);
  pnh.param("zone/clear_radius",    a.zone.clear_radius,    a.zone.clear_radius);
  pnh.param("zone/clear_height",    a.zone.clear_height,    a.zone.clear_height);
  pnh.param("zone/max_slope_deg",   a.zone.max_slope_deg,   a.zone.max_slope_deg);
  pnh.param("zone/max_roughness",   a.zone.max_roughness,   a.zone.max_roughness);
  pnh.param("zone/min_inliers",     a.zone.min_inliers,     a.zone.min_inliers);
  pnh.param("zone/max_clear_pts",   a.zone.max_clear_pts,   a.zone.max_clear_pts);
  pnh.param("zone/ransac_thresh",   a.zone.ransac_thresh,   a.zone.ransac_thresh);
  pnh.param("zone/search_radius",   a.zone.search_radius,   a.zone.search_radius);
  pnh.param("zone/search_step",     a.zone.search_step,     a.zone.search_step);
  pnh.param("zone/z_band_lo",       a.zone.z_band_lo,       a.zone.z_band_lo);
  pnh.param("zone/z_band_hi",       a.zone.z_band_hi,       a.zone.z_band_hi);

  pnh.param("descent/vz_nominal",   a.descent.vz_nominal,   a.descent.vz_nominal);
  pnh.param("descent/vz_slow",      a.descent.vz_slow,      a.descent.vz_slow);
  pnh.param("descent/slow_dist",    a.descent.slow_dist,    a.descent.slow_dist);
  pnh.param("descent/stop_dist",    a.descent.stop_dist,    a.descent.stop_dist);
  pnh.param("descent/abort_dist",   a.descent.abort_dist,   a.descent.abort_dist);
  pnh.param("descent/abort_hold_s", a.descent.abort_hold_s, a.descent.abort_hold_s);
  pnh.param("descent/cone_half_angle_deg",
            a.descent.cone_half_angle_deg, a.descent.cone_half_angle_deg);
  pnh.param("descent/cone_radius",  a.descent.cone_radius,  a.descent.cone_radius);
  pnh.param("descent/target_z",     a.descent.target_z,     a.descent.target_z);

  pnh.param("precision/align_tol_xy",    a.precision.align_tol_xy,    a.precision.align_tol_xy);
  pnh.param("precision/align_timeout_s", a.precision.align_timeout_s, a.precision.align_timeout_s);
  pnh.param("precision/kp_xy",           a.precision.kp_xy,           a.precision.kp_xy);
  pnh.param("precision/max_v_xy",        a.precision.max_v_xy,        a.precision.max_v_xy);
  pnh.param("precision/handoff_z_agl",   a.precision.handoff_z_agl,   a.precision.handoff_z_agl);
  a.precision.use_marker   = a.use_marker;
  a.precision.marker_topic = a.marker_topic;
  a.precision.marker_id    = a.marker_id;

  loadWaypoints(pnh, a.waypoints);
}

static std::vector<Waypoint> buildAbsoluteWaypoints(const std::vector<Waypoint>& rel_wps,
                                                    double init_x,
                                                    double init_y,
                                                    double init_z,
                                                    double init_yaw,
                                                    double cruise_height,
                                                    bool zero_z_means_current_z) {
  std::vector<Waypoint> abs_wps;
  const double cy = std::cos(init_yaw), sy = std::sin(init_yaw);
  abs_wps.reserve(rel_wps.size());
  for (const auto& w : rel_wps) {
    Waypoint a = w;
    a.x = init_x + cy * w.x - sy * w.y;
    a.y = init_y + sy * w.x + cy * w.y;
    // Standalone mode captures init_z on the ground, so z=0 means cruise height
    // AGL. External-control mode captures init_z after mission_manager takeoff,
    // so z=0 should keep the current flight altitude.
    if (std::abs(w.z) < 1e-6) {
      a.z = zero_z_means_current_z ? init_z : init_z + cruise_height;
    } else {
      a.z = init_z + w.z;
    }
    a.yaw = init_yaw + w.yaw;
    abs_wps.push_back(a);
  }
  return abs_wps;
}

enum class Stage {
  WaitEnabled, WaitOdom, Takeoff, Cruise,
  Hover, Evaluate, MoveToSafe,
  Descend, Precision, Done, Failed
};

static const char* stageName(Stage s) {
  switch (s) {
    case Stage::WaitEnabled: return "WaitEnabled";
    case Stage::WaitOdom:   return "WaitOdom";
    case Stage::Takeoff:    return "Takeoff";
    case Stage::Cruise:     return "Cruise";
    case Stage::Hover:      return "Hover";
    case Stage::Evaluate:   return "Evaluate";
    case Stage::MoveToSafe: return "MoveToSafe";
    case Stage::Descend:    return "Descend";
    case Stage::Precision:  return "Precision";
    case Stage::Done:       return "Done";
    case Stage::Failed:     return "Failed";
  }
  return "?";
}

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "safe_landing_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  AppParams ap; loadParams(pnh, ap);
  if (ap.external_control && ap.cruise_only) {
    ROS_WARN("safe_landing_node: external_control+cruise_only enabled; "
             "node will wait for ~enabled, skip takeoff/landing, and publish ~done");
  }

  PX4Interface px4(nh);
  GridPlanner planner(nh, ap.cloud_topic, ap.planner);
  WaypointFollower wpf(nh, px4, ap.cloud_topic, ap.waypoint);
  LandingZoneEvaluator zone(nh, ap.cloud_topic, ap.zone);
  GuardedDescent descent(nh, px4, ap.cloud_topic, ap.descent);
  PrecisionLanding precision(nh, px4, ap.precision);
  auto externalCanceled = [&pnh, &ap]() {
    if (!ap.external_control) return false;
    bool enabled = false;
    pnh.param("enabled", enabled, false);
    return !enabled;
  };
  if (ap.external_control) {
    wpf.setCancelCallback(externalCanceled);
  }

  FlightLogger flog(ap.log_dir);
  wpf.setLogger(&flog);
  descent.setLogger(&flog);

  ros::Publisher done_pub = pnh.advertise<std_msgs::Bool>("done", 1, true);
  ros::Publisher failed_pub = pnh.advertise<std_msgs::Bool>("failed", 1, true);
  auto publishBool = [](ros::Publisher& pub, bool value) {
    std_msgs::Bool msg;
    msg.data = value;
    pub.publish(msg);
  };
  publishBool(done_pub, false);
  publishBool(failed_pub, false);

  ros::Rate rate(30.0);
  Stage stage = (ap.external_control && !ap.enabled) ? Stage::WaitEnabled : Stage::WaitOdom;
  ros::Time stage_start = ros::Time::now();

  // Initial pose, captured at takeoff. Waypoints are interpreted as
  // (initial_x + dx_yaw_rotated) so the user writes them in the
  // body-aligned takeoff frame, just like microuav2025_api does.
  double init_x = 0, init_y = 0, init_z = 0, init_yaw = 0;
  double ground_z_for_planning = 0;
  bool   init_captured = false;

  ZoneResult zone_result;
  std::vector<Waypoint> cruise_goals;
  // Where the cruise actually ended (for fall-back land target).
  double cruise_end_x = 0, cruise_end_y = 0;

  ROS_INFO("safe_landing_node up: cruise_h=%.2f, %zu waypoints, marker=%s",
           ap.cruise_height, ap.waypoints.size(), ap.use_marker ? "on" : "off");

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const double t_in_stage = (now - stage_start).toSec();

    if (ap.external_control) {
      bool enabled_now = false;
      pnh.param("enabled", enabled_now, false);
      if (!enabled_now) {
        if (stage != Stage::WaitEnabled) {
          ROS_INFO("safe_landing_node: disabled by external controller");
          stage = Stage::WaitEnabled;
          stage_start = now;
        }
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      if (stage == Stage::WaitEnabled) {
        publishBool(done_pub, false);
        publishBool(failed_pub, false);
        init_captured = false;
        cruise_goals.clear();
        stage = Stage::WaitOdom;
        stage_start = now;
        ROS_INFO("safe_landing_node: enabled by external controller");
      }
    }

    // Flight log: tag stage transitions, sample telemetry each tick. Cruise /
    // Descend sample themselves (with avoidance fields) inside their modules.
    static Stage prev_stage = Stage::WaitOdom;
    if (stage != prev_stage) {
      flog.event(stageName(stage), px4, "enter_stage");
      prev_stage = stage;
    }
    if (stage != Stage::Cruise && stage != Stage::Descend) {
      flog.sample(stageName(stage), px4);
    }

    switch (stage) {
      // ---------------------------------------------------------------------
      case Stage::WaitEnabled:
        ROS_INFO_THROTTLE(2, "waiting for external ~enabled=true ...");
        break;

      // ---------------------------------------------------------------------
      case Stage::WaitOdom:
        if (px4.poseReady() && px4.connected()) {
          init_x = px4.x(); init_y = px4.y();
          init_z = px4.z(); init_yaw = px4.yaw();
          ground_z_for_planning =
              (ap.external_control && ap.cruise_only)
                  ? (init_z - ap.cruise_height)
                  : init_z;
          init_captured = true;
          ROS_INFO("init pose: (%.2f, %.2f, %.2f) yaw=%.2f",
                   init_x, init_y, init_z, init_yaw);
          if (ap.external_control && ap.cruise_only) {
            cruise_goals = buildAbsoluteWaypoints(
                ap.waypoints, init_x, init_y, init_z, init_yaw,
                ap.cruise_height, true);
            wpf.setWaypoints(cruise_goals);
            stage = Stage::Cruise; stage_start = now;
          } else {
            stage = Stage::Takeoff; stage_start = now;
          }
        } else {
          ROS_INFO_THROTTLE(1, "waiting for odom & mavros connection ...");
        }
        break;

      // ---------------------------------------------------------------------
      case Stage::Takeoff: {
        px4.setPosition(init_x, init_y,
                        init_z + ap.cruise_height, init_yaw);
        if (t_in_stage < 0.5) {
          if (!px4.setOffboardAndArm(ap.takeoff_timeout)) {
            ROS_ERROR("takeoff: failed to enter OFFBOARD/arm");
            stage = Stage::Failed; stage_start = now; break;
          }
          ROS_INFO("OFFBOARD+armed");
        }
        const double err = std::abs(px4.z() - (init_z + ap.cruise_height));
        if (err < 0.15) {
          ROS_INFO("takeoff complete (z err %.2f)", err);
          // Build absolute waypoint list by rotating user dx/dy around init_yaw.
          // In standalone mode init_z is the ground height, so z=0 means
          // cruise_height AGL.
          cruise_goals = buildAbsoluteWaypoints(
              ap.waypoints, init_x, init_y, init_z, init_yaw,
              ap.cruise_height, false);
          wpf.setWaypoints(cruise_goals);
          stage = Stage::Cruise; stage_start = now;
        } else if (t_in_stage > ap.takeoff_timeout + 5.0) {
          ROS_ERROR("takeoff timeout (z err %.2f)", err);
          stage = Stage::Failed; stage_start = now;
        }
        break;
      }

      // ---------------------------------------------------------------------
      case Stage::Cruise: {
        WaypointResult wr;
        if (ap.planner.enabled) {
          if (!planner.haveCloud()) {
            if (t_in_stage <= ap.cloud_wait_timeout) {
              ROS_WARN_THROTTLE(
                  1.0,
                  "planner: waiting for cloud before cruise (%.1f/%.1fs)",
                  t_in_stage, ap.cloud_wait_timeout);
              px4.holdHere();
              px4.publish();
              break;
            }
            ROS_ERROR("planner: no cloud available before cruise");
            stage = Stage::Failed; stage_start = ros::Time::now();
            break;
          }
          wr = runRollingCruise(cruise_goals, planner, wpf, px4, ap.planner,
                                ground_z_for_planning, &flog, externalCanceled);
        } else {
          wr = wpf.run();
        }
        cruise_end_x = wpf.endX();
        cruise_end_y = wpf.endY();
        // Guard against the follower never reaching/recording a leg (returns 0,0):
        // fall back to where we actually are.
        if (cruise_end_x == 0.0 && cruise_end_y == 0.0) {
          cruise_end_x = px4.x();
          cruise_end_y = px4.y();
        }
        if (wr == WaypointResult::AllReached || wr == WaypointResult::PartialSkipped) {
          if (wr == WaypointResult::PartialSkipped) {
            ROS_WARN("cruise: some waypoints skipped, attempting landing anyway");
          }
          if (ap.external_control && ap.cruise_only) {
            ROS_INFO("cruise complete — reporting done to external controller");
            stage = Stage::Done; stage_start = ros::Time::now();
          } else {
            stage = Stage::Hover; stage_start = ros::Time::now();
          }
        } else {
          ROS_ERROR("cruise failed: %d", static_cast<int>(wr));
          stage = Stage::Failed; stage_start = ros::Time::now();
        }
        break;
      }

      // ---------------------------------------------------------------------
      case Stage::Hover:
        px4.setPosition(px4.x(), px4.y(),
                        init_z + ap.cruise_height, px4.yaw());
        if (t_in_stage > 1.5 && zone.haveCloud()) {
          stage = Stage::Evaluate; stage_start = now;
        } else if (t_in_stage > 5.0) {
          ROS_ERROR("hover: still no cloud after 5s");
          stage = Stage::Failed; stage_start = now;
        }
        break;

      // ---------------------------------------------------------------------
      case Stage::Evaluate: {
        const double tx = std::isnan(ap.land_x) ? cruise_end_x : ap.land_x;
        const double ty = std::isnan(ap.land_y) ? cruise_end_y : ap.land_y;
        zone_result = zone.evaluate(tx, ty, init_z);
        if (zone_result.ok) {
          stage = Stage::MoveToSafe; stage_start = now;
        } else {
          ROS_ERROR("no safe landing zone — retrying once after 2s");
          holdFor(px4, 2.0);
          zone_result = zone.evaluate(tx, ty, init_z);
          if (zone_result.ok) {
            stage = Stage::MoveToSafe; stage_start = now;
          } else {
            stage = Stage::Failed; stage_start = now;
          }
        }
        break;
      }

      // ---------------------------------------------------------------------
      case Stage::MoveToSafe: {
        // Drive to the safe zone through the reactive follower (not a blind
        // position setpoint) so we still avoid obstacles on the way.
        Waypoint w;
        w.x = zone_result.x; w.y = zone_result.y;
        w.z = init_z + ap.cruise_height; w.yaw = px4.yaw();
        w.tol = 0.10; w.v_max = 0.4;
        wpf.setWaypoints({w});
        wpf.run();
        const double d = std::hypot(zone_result.x - px4.x(),
                                    zone_result.y - px4.y());
        ROS_INFO("move-to-safe done (d=%.2f), descending", d);
        stage = Stage::Descend; stage_start = ros::Time::now();
        break;
      }

      // ---------------------------------------------------------------------
      case Stage::Descend: {
        DescentResult res = descent.run(zone_result.z, 30.0);
        if (res == DescentResult::Reached) {
          stage = Stage::Precision; stage_start = ros::Time::now();
        } else if (res == DescentResult::Aborted) {
          // Don't AUTO.LAND here — there's an obstacle below us.
          // Climb back up and try a fresh evaluation once.
          ROS_WARN("descend aborted, climbing back to re-evaluate");
          ros::Time t0 = ros::Time::now();
          while ((ros::Time::now() - t0).toSec() < 4.0 && ros::ok()) {
            px4.setVelocity(0.0, 0.0, 0.4, px4.yaw());
            px4.publish();
            ros::spinOnce();
            rate.sleep();
          }
          stage = Stage::Hover; stage_start = ros::Time::now();
        } else {
          ROS_ERROR("descent failed: %d", static_cast<int>(res));
          stage = Stage::Failed; stage_start = ros::Time::now();
        }
        break;
      }

      // ---------------------------------------------------------------------
      case Stage::Precision: {
        PrecisionResult res = precision.run(zone_result.z);
        if (res == PrecisionResult::Landed) {
          stage = Stage::Done; stage_start = ros::Time::now();
        } else {
          // Not confirmed on the ground — let AUTO.LAND finish touchdown
          // rather than declaring success.
          ROS_WARN("precision did not confirm touchdown (%d), handing to AUTO.LAND",
                   static_cast<int>(res));
          stage = Stage::Failed; stage_start = ros::Time::now();
        }
        break;
      }

      case Stage::Done:
        if (ap.external_control) {
          ROS_INFO_THROTTLE(2, "DONE — external controller may take over");
          publishBool(done_pub, true);
          pnh.setParam("enabled", false);
          stage = Stage::WaitEnabled;
          stage_start = ros::Time::now();
          break;
        }
        ROS_INFO_THROTTLE(2, "DONE — task finished");
        ros::Duration(1.0).sleep();
        ros::shutdown();
        return 0;

      case Stage::Failed:
        if (ap.external_control) {
          ROS_ERROR_THROTTLE(2, "FAILED — reporting failure to external controller");
          publishBool(failed_pub, true);
          pnh.setParam("enabled", false);
          stage = Stage::WaitEnabled;
          stage_start = ros::Time::now();
          break;
        }
        ROS_ERROR_THROTTLE(2, "FAILED — handing off to AUTO.LAND");
        px4.setMode("AUTO.LAND");
        ros::Duration(2.0).sleep();
        ros::shutdown();
        return 1;
    }

    if (stage != Stage::WaitEnabled &&
        stage != Stage::Cruise && stage != Stage::Descend &&
        stage != Stage::Precision && stage != Stage::Done &&
        stage != Stage::Failed) {
      px4.publish();
    }
    ROS_INFO_THROTTLE(2, "[stage=%s t=%.1fs pos=(%.2f,%.2f,%.2f)]",
                      stageName(stage), t_in_stage,
                      px4.x(), px4.y(), px4.z());

    ros::spinOnce();
    rate.sleep();
  }
  (void)init_captured;
  return 0;
}
