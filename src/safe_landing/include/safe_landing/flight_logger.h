#pragma once

#include "safe_landing/px4_interface.h"

#include <ros/ros.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace safe_landing {

// Lightweight CSV flight recorder for post-flight analysis.
//
// One wide row per sample. Columns the caller doesn't have (e.g. obstacle
// distances outside of cruise) are written as NaN so the schema stays fixed
// and is trivial to load with pandas:  df = pd.read_csv(path)
//
// All access is from the single FSM thread, so no locking is needed.
class FlightLogger {
 public:
  // dir: directory to write into (created if missing). A timestamped file
  // flight_YYYYmmdd_HHMMSS.csv is opened inside it.
  explicit FlightLogger(const std::string& dir) {
    ::mkdir(dir.c_str(), 0775);  // ok if it already exists
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
    path_ = dir + "/flight_" + stamp + ".csv";
    f_.open(path_, std::ios::out | std::ios::trunc);
    if (f_.is_open()) {
      f_ << "t_rel,t_ros,stage,event,"
            "x,y,z,yaw,vx,vy,vz,speed_xy,"
            "d_fwd,d_side,scale,goal_x,goal_y\n";
      f_.flush();
      t0_ = ros::Time::now();
      ROS_INFO("FlightLogger: recording to %s", path_.c_str());
    } else {
      ROS_ERROR("FlightLogger: could not open %s — logging disabled",
                path_.c_str());
    }
  }

  bool ok() const { return f_.is_open(); }
  const std::string& path() const { return path_; }

  // Plain telemetry sample (no cruise/avoidance fields).
  void sample(const std::string& stage, const PX4Interface& px4) {
    write(stage, "", px4, kNaN, kNaN, kNaN, kNaN, kNaN);
  }

  // Cruise sample: includes reactive-avoidance state.
  void sampleCruise(const std::string& stage, const PX4Interface& px4,
                    double d_fwd, double d_side, double scale,
                    double goal_x, double goal_y) {
    write(stage, "", px4, d_fwd, d_side, scale, goal_x, goal_y);
  }

  // Descent sample: d_fwd column carries the below-obstacle distance.
  void sampleDescent(const std::string& stage, const PX4Interface& px4,
                     double d_obstacle) {
    write(stage, "", px4, d_obstacle, kNaN, kNaN, kNaN, kNaN);
  }

  // Tagged event (stage transition, replan, blocked, skip, abort, ...).
  void event(const std::string& stage, const PX4Interface& px4,
             const std::string& msg) {
    write(stage, msg, px4, kNaN, kNaN, kNaN, kNaN, kNaN);
  }

 private:
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  static std::string num(double v) {
    if (std::isnan(v)) return "";        // empty cell == NaN in pandas
    std::ostringstream os;
    os << v;
    return os.str();
  }

  void write(const std::string& stage, const std::string& event,
             const PX4Interface& px4, double d_fwd, double d_side,
             double scale, double goal_x, double goal_y) {
    if (!f_.is_open()) return;
    const ros::Time now = ros::Time::now();
    f_ << (now - t0_).toSec() << ',' << now.toSec() << ','
       << stage << ',' << event << ','
       << num(px4.x()) << ',' << num(px4.y()) << ',' << num(px4.z()) << ','
       << num(px4.yaw()) << ','
       << num(px4.vx()) << ',' << num(px4.vy()) << ',' << num(px4.vz()) << ','
       << num(px4.speedXY()) << ','
       << num(d_fwd) << ',' << num(d_side) << ',' << num(scale) << ','
       << num(goal_x) << ',' << num(goal_y) << '\n';
    f_.flush();  // survive a crash / hard kill
  }

  std::ofstream f_;
  std::string path_;
  ros::Time t0_;
};

}  // namespace safe_landing
