// 滚动局部障碍地图 (rolling local obstacle map)
// ---------------------------------------------------------------------------
// 目的:补上原架构的方法硬伤——grid_planner / waypoint_follower 都只用
// 【最新一帧】 /cloud_registered,且没有任何障碍记忆。这导致:
//   1) Livox Mid360 单帧稀疏,细树干点太少被聚类滤掉;
//   2) 一棵树正后方的树被遮挡(无回波)→ 规划器根本不知道它存在
//      → “两棵树排成一条线避不了,错开就能避”。
//
// 本节点夹在 /cloud_registered 与消费者之间,只做点云层面的累积+裁剪,
// 不碰任何规划/控制逻辑(消费者代码零改动,只需把 cloud_topic 指过来):
//   · 累积最近 max_age 秒的帧  → 把稀疏单帧变密 + 记住刚被遮挡的障碍;
//   · 以无人机为心裁掉 max_range 外的点、并限制帧数/帧龄
//                              → 把定位漂移的“拖影”和内存/算力都框住。
// “要记住被遮挡的树”和“要忘掉漂移的拖影”这对矛盾,正好用一个有界滚动窗口折中。
//
// 输入: ~input_topic   (默认 /cloud_registered, 世界/odom 系)
//        ~odom_topic    (默认 /Odometry, 与点云同系, 用于按距离裁剪)
// 输出: ~output_topic  (默认 /cloud_local_map, 同 frame_id, 直接喂给 safe_landing)

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <deque>
#include <mutex>
#include <string>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Frame {
  ros::Time stamp;
  Cloud::Ptr cloud;
};

class LocalObstacleMap {
 public:
  LocalObstacleMap(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("input_topic", input_topic_, "/cloud_registered");
    pnh.param<std::string>("output_topic", output_topic_, "/cloud_local_map");
    pnh.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh.param("max_age", max_age_, 1.5);        // [s]  记忆时长:越大越能记住被遮挡的树,但漂移拖影越重
    pnh.param("max_range", max_range_, 6.0);    // [m]  以机为心的水平保留半径
    pnh.param("voxel_leaf", voxel_leaf_, 0.10); // [m]  体素下采样,去重+控点数
    pnh.param("max_frames", max_frames_, 40);   // 帧数硬上限,防雷达高频时爆内存

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    cloud_sub_ = nh.subscribe(input_topic_, 4, &LocalObstacleMap::cloudCb, this);
    odom_sub_ = nh.subscribe(odom_topic_, 10, &LocalObstacleMap::odomCb, this);

    ROS_INFO("local_obstacle_map: %s (+%s) -> %s | max_age=%.2fs max_range=%.1fm "
             "voxel=%.2fm max_frames=%d",
             input_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str(),
             max_age_, max_range_, voxel_leaf_, max_frames_);
  }

 private:
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    drone_x_ = msg->pose.pose.position.x;
    drone_y_ = msg->pose.pose.position.y;
    have_odom_ = true;
  }

  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 1) 单帧体素下采样
    Cloud::Ptr raw(new Cloud);
    pcl::fromROSMsg(*msg, *raw);
    if (raw->empty()) return;

    Cloud::Ptr frame(new Cloud);
    const float leaf = static_cast<float>(std::max(0.02, voxel_leaf_));
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*frame);

    double cx, cy;
    bool have_odom;
    {
      std::lock_guard<std::mutex> lk(mu_);
      frames_.push_back({msg->header.stamp, frame});
      // 2) 按帧龄 / 帧数淘汰旧帧
      const ros::Time newest = msg->header.stamp;
      while (!frames_.empty() &&
             (newest - frames_.front().stamp).toSec() > max_age_) {
        frames_.pop_front();
      }
      while (static_cast<int>(frames_.size()) > max_frames_) {
        frames_.pop_front();
      }
      cx = drone_x_;
      cy = drone_y_;
      have_odom = have_odom_;
    }

    // 3) 合并所有保留帧,并按距无人机的水平距离裁剪
    Cloud::Ptr merged(new Cloud);
    const double r2 = max_range_ * max_range_;
    {
      std::lock_guard<std::mutex> lk(mu_);
      for (const auto& f : frames_) {
        for (const auto& pt : f.cloud->points) {
          if (have_odom) {
            const double dx = pt.x - cx, dy = pt.y - cy;
            if (dx * dx + dy * dy > r2) continue;
          }
          merged->push_back(pt);
        }
      }
    }
    if (merged->empty()) return;

    // 4) 对合并结果再体素一次,去掉重叠帧的重复点
    Cloud::Ptr out(new Cloud);
    vg.setInputCloud(merged);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*out, out_msg);
    out_msg.header.frame_id = msg->header.frame_id;  // 与输入同系,消费者无需改坐标
    out_msg.header.stamp = msg->header.stamp;
    pub_.publish(out_msg);

    ROS_INFO_THROTTLE(2.0,
        "local_obstacle_map: frames=%zu in=%zu -> out=%zu pts (odom=%s)",
        frames_.size(), raw->size(), out->size(), have_odom ? "ok" : "WAIT");
  }

  std::string input_topic_, output_topic_, odom_topic_;
  double max_age_{1.5}, max_range_{6.0}, voxel_leaf_{0.10};
  int max_frames_{40};

  ros::Publisher pub_;
  ros::Subscriber cloud_sub_, odom_sub_;

  std::mutex mu_;
  std::deque<Frame> frames_;
  double drone_x_{0.0}, drone_y_{0.0};
  bool have_odom_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_obstacle_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  LocalObstacleMap node(nh, pnh);
  ros::spin();
  return 0;
}
