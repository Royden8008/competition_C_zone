#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "safe_landing/waypoint_follower.h"

#include <mutex>
#include <string>
#include <vector>

namespace safe_landing {

struct PlannerParams {
  bool enabled = true;
  double resolution = 0.20;
  double inflation_radius = 0.60;
  double xy_margin = 1.50;
  double obstacle_min_z_agl = 0.15;
  double obstacle_max_z_agl = 2.00;
  double waypoint_spacing = 0.80;
  // Tracking tightness for *intermediate* path points (the final goal keeps its
  // own tol/v_max). Smaller tol forces the follower to stay on the planned
  // corridor instead of cutting the corner into the inflation margin; the
  // lower v_max shrinks the turn radius (~v^2/accel) so corners round less.
  double waypoint_tol = 0.10;     // [m]
  double waypoint_v_max = 0.40;   // [m/s]
  int max_plan_points = 80;
  int execute_points = 2;
  int max_replans = 20;
  double scan_yaw_step_deg = 30.0;
  double scan_max_yaw_deg = 120.0;
  double scan_settle_s = 0.35;
  double relaxed_inflation_radius = 0.40;
  double relaxed_xy_margin = 2.50;
  bool cluster_filter_enabled = true;
  double cluster_tolerance = 0.35;
  int cluster_min_points = 8;
  int cluster_max_points = 20000;
};

class GridPlanner {
 public:
  GridPlanner(ros::NodeHandle& nh,
              const std::string& cloud_topic,
              const PlannerParams& params);

  bool haveCloud() const;
  bool plan(double start_x,
            double start_y,
            double ground_z,
            const Waypoint& goal,
            std::vector<Waypoint>& out) const;
  bool plan(double start_x,
            double start_y,
            double ground_z,
            const Waypoint& goal,
            double inflation_radius,
            double xy_margin,
            std::vector<Waypoint>& out) const;

 private:
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);

  ros::Subscriber cloud_sub_;
  PlannerParams p_;

  mutable std::mutex mu_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  // Last successful path (world XY), used to bias replans toward the same
  // detour side and kill left/right oscillation.
  mutable std::vector<std::pair<double, double>> last_path_;
};

}  // namespace safe_landing
