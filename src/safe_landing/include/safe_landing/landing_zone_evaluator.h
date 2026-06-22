#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <vector>

namespace safe_landing {

struct ZoneParams {
  // Search disc around the requested landing target (world XY).
  double disc_radius      = 0.40;   // [m] points considered for plane fit
  double clear_radius     = 0.30;   // [m] horizontal clearance check
  double clear_height     = 0.50;   // [m] vertical clearance above ground
  // Quality thresholds.
  double max_slope_deg    = 8.0;    // [deg] tilt of the fitted plane vs +Z
  double max_roughness    = 0.03;   // [m]   RMS of inlier residuals
  int    min_inliers      = 80;     // points required for a confident fit
  int    max_clear_pts    = 5;      // points allowed in the clearance cylinder
  double ransac_thresh    = 0.02;   // [m]   RANSAC inlier threshold
  // Grid search if the requested point fails.
  double search_radius    = 0.60;   // [m]   how far to search around target
  double search_step      = 0.20;   // [m]   grid resolution
  // Cloud is assumed in world frame; only points within the height band
  // [target_z - z_band_lo, target_z + z_band_hi] are considered "ground".
  double z_band_lo        = 0.30;
  double z_band_hi        = 1.50;
};

struct ZoneResult {
  bool    ok = false;
  double  x = 0.0, y = 0.0, z = 0.0;     // centroid of the chosen patch
  double  slope_deg  = 0.0;
  double  roughness  = 0.0;
  int     inliers    = 0;
  int     clearance_violations = 0;
  std::string reason;                     // populated when !ok
};

// Subscribes to a registered LiDAR cloud (e.g. FAST-LIO /cloud_registered)
// and answers "is there a flat, clear patch I can land on near (x,y)?".
// Pure-evaluator — does not command motion.
class LandingZoneEvaluator {
 public:
  LandingZoneEvaluator(ros::NodeHandle& nh,
                       const std::string& cloud_topic,
                       const ZoneParams& params);

  // Evaluate the requested target, falling back to a small grid search.
  // target_z is the expected ground height (you can pass takeoff_z - height_AGL).
  ZoneResult evaluate(double target_x, double target_y, double target_z);

  bool haveCloud() const;

 private:
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
  ZoneResult evaluatePatch(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                           double cx, double cy, double cz) const;
  void publishMarker(const ZoneResult& r) const;

  ros::Subscriber cloud_sub_;
  mutable ros::Publisher marker_pub_;
  ZoneParams p_;

  mutable std::mutex mu_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  ros::Time cloud_stamp_;
};

}  // namespace safe_landing
