#include "safe_landing/grid_planner.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace safe_landing {

namespace {

struct Node {
  int idx = 0;
  double f = 0.0;
};

struct NodeGreater {
  bool operator()(const Node& a, const Node& b) const {
    return a.f > b.f;
  }
};

double dist2d(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

// True if a straight grid line from (x0,y0) to (x1,y1) crosses no occupied
// cell (Bresenham). Used for string-pulling path smoothing.
bool lineOfSight(int x0, int y0, int x1, int y1, int w,
                 const std::vector<unsigned char>& occ) {
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  while (true) {
    if (occ[y0 * w + x0]) return false;
    if (x0 == x1 && y0 == y1) return true;
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
}

}  // namespace

GridPlanner::GridPlanner(ros::NodeHandle& nh,
                         const std::string& cloud_topic,
                         const PlannerParams& params)
    : p_(params), cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
  cloud_sub_ = nh.subscribe(cloud_topic, 2, &GridPlanner::cloudCb, this);
}

bool GridPlanner::haveCloud() const {
  std::lock_guard<std::mutex> lk(mu_);
  return cloud_ && !cloud_->empty();
}

void GridPlanner::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);
  pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
  const float leaf = static_cast<float>(std::max(0.05, p_.resolution * 0.5));
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*c);
  std::lock_guard<std::mutex> lk(mu_);
  cloud_ = c;
}

bool GridPlanner::plan(double start_x,
                       double start_y,
                       double ground_z,
                       const Waypoint& goal,
                       std::vector<Waypoint>& out) const {
  return plan(start_x, start_y, ground_z, goal,
              p_.inflation_radius, p_.xy_margin, out);
}

bool GridPlanner::plan(double start_x,
                       double start_y,
                       double ground_z,
                       const Waypoint& goal,
                       double inflation_radius,
                       double xy_margin,
                       std::vector<Waypoint>& out) const {
  out.clear();

  pcl::PointCloud<pcl::PointXYZ>::Ptr snap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap = cloud_;
  }
  if (!snap || snap->empty()) return false;

  const double res = std::max(0.05, p_.resolution);
  const double min_x = std::min(start_x, goal.x) - xy_margin;
  const double max_x = std::max(start_x, goal.x) + xy_margin;
  const double min_y = std::min(start_y, goal.y) - xy_margin;
  const double max_y = std::max(start_y, goal.y) + xy_margin;
  const int w = static_cast<int>(std::ceil((max_x - min_x) / res)) + 1;
  const int h = static_cast<int>(std::ceil((max_y - min_y) / res)) + 1;
  if (w <= 2 || h <= 2 || static_cast<long long>(w) * h > 1000000) return false;

  auto toIndex = [w](int ix, int iy) { return iy * w + ix; };
  auto toIx = [min_x, res](double x) { return static_cast<int>(std::round((x - min_x) / res)); };
  auto toIy = [min_y, res](double y) { return static_cast<int>(std::round((y - min_y) / res)); };
  auto cellX = [min_x, res](int ix) { return min_x + ix * res; };
  auto cellY = [min_y, res](int iy) { return min_y + iy * res; };
  auto inside = [w, h](int ix, int iy) { return ix >= 0 && iy >= 0 && ix < w && iy < h; };

  std::vector<unsigned char> occupied(w * h, 0);
  const int inflate = static_cast<int>(std::ceil(inflation_radius / res));
  const double min_z = ground_z + p_.obstacle_min_z_agl;
  const double max_z = ground_z + p_.obstacle_max_z_agl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>);
  candidates->reserve(snap->size());
  for (const auto& pt : snap->points) {
    if (pt.z < min_z || pt.z > max_z) continue;
    if (pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y) continue;
    candidates->push_back(pt);
  }

  std::vector<const pcl::PointXYZ*> obstacle_points;
  if (p_.cluster_filter_enabled && !candidates->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(candidates);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p_.cluster_tolerance);
    ec.setMinClusterSize(std::max(1, p_.cluster_min_points));
    ec.setMaxClusterSize(std::max(p_.cluster_min_points, p_.cluster_max_points));
    ec.setSearchMethod(tree);
    ec.setInputCloud(candidates);
    ec.extract(cluster_indices);

    size_t kept = 0;
    for (const auto& cluster : cluster_indices) {
      kept += cluster.indices.size();
      for (int idx : cluster.indices) obstacle_points.push_back(&(*candidates)[idx]);
    }
    ROS_INFO_THROTTLE(1, "planner: obstacle candidates=%zu clusters=%zu kept_points=%zu",
                      candidates->size(), cluster_indices.size(), kept);
  } else {
    obstacle_points.reserve(candidates->size());
    for (const auto& pt : candidates->points) obstacle_points.push_back(&pt);
    ROS_INFO_THROTTLE(1, "planner: obstacle candidates=%zu cluster_filter=off", candidates->size());
  }

  for (const auto* pt : obstacle_points) {
    const int cx = toIx(pt->x);
    const int cy = toIy(pt->y);
    if (!inside(cx, cy)) continue;
    for (int dy = -inflate; dy <= inflate; ++dy) {
      for (int dx = -inflate; dx <= inflate; ++dx) {
        if (dx * dx + dy * dy > inflate * inflate) continue;
        const int ix = cx + dx;
        const int iy = cy + dy;
        if (inside(ix, iy)) occupied[toIndex(ix, iy)] = 1;
      }
    }
  }

  const int sx = toIx(start_x);
  const int sy = toIy(start_y);
  const int gx = toIx(goal.x);
  const int gy = toIy(goal.y);
  if (!inside(sx, sy) || !inside(gx, gy)) return false;

  const int start = toIndex(sx, sy);
  const int finish = toIndex(gx, gy);
  occupied[start] = 0;
  occupied[finish] = 0;

  std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
  std::vector<int> parent(w * h, -1);
  std::priority_queue<Node, std::vector<Node>, NodeGreater> open;

  // Hysteresis corridor: mark cells near the previous successful path. A* pays
  // a small extra cost to leave this corridor, so replans prefer the same
  // detour side instead of flip-flopping left/right (oscillation).
  std::vector<unsigned char> near_last;
  {
    std::vector<std::pair<double, double>> prev;
    {
      std::lock_guard<std::mutex> lk(mu_);
      prev = last_path_;
    }
    if (!prev.empty()) {
      near_last.assign(w * h, 0);
      const int band = std::max(1, static_cast<int>(std::round(0.40 / res)));
      for (const auto& pxy : prev) {
        const int px = toIx(pxy.first);
        const int py = toIy(pxy.second);
        for (int dy = -band; dy <= band; ++dy)
          for (int dx = -band; dx <= band; ++dx) {
            const int ix2 = px + dx, iy2 = py + dy;
            if (inside(ix2, iy2)) near_last[toIndex(ix2, iy2)] = 1;
          }
      }
    }
  }
  // Cost (in meters) added when stepping outside the previous corridor.
  const double off_corridor_cost = res * 0.6;

  g_score[start] = 0.0;
  open.push({start, dist2d(start_x, start_y, goal.x, goal.y)});

  constexpr int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                              {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();
    if (cur.idx == finish) break;

    const int ix = cur.idx % w;
    const int iy = cur.idx / w;
    for (const auto& d : dirs) {
      const int nx = ix + d[0];
      const int ny = iy + d[1];
      if (!inside(nx, ny)) continue;
      const int ni = toIndex(nx, ny);
      if (occupied[ni]) continue;
      double step = (d[0] != 0 && d[1] != 0) ? res * std::sqrt(2.0) : res;
      if (!near_last.empty() && !near_last[ni]) step += off_corridor_cost;
      const double tentative = g_score[cur.idx] + step;
      if (tentative >= g_score[ni]) continue;
      parent[ni] = cur.idx;
      g_score[ni] = tentative;
      const double f = tentative + dist2d(cellX(nx), cellY(ny), goal.x, goal.y);
      open.push({ni, f});
    }
  }

  if (parent[finish] < 0 && finish != start) return false;

  std::vector<int> path;
  for (int at = finish; at >= 0; at = parent[at]) {
    path.push_back(at);
    if (at == start) break;
  }
  if (path.empty() || path.back() != start) return false;
  std::reverse(path.begin(), path.end());

  // String-pulling: keep only turning points where the straight line to the
  // next kept point would clip an obstacle. Turns a 45-degree zig-zag into a
  // few long straight segments.
  std::vector<int> waypts;
  waypts.push_back(path.front());
  size_t anchor = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    const int ax = path[anchor] % w, ay = path[anchor] / w;
    const int cx = path[i] % w,      cy = path[i] / w;
    if (!lineOfSight(ax, ay, cx, cy, w, occupied)) {
      waypts.push_back(path[i - 1]);   // last point still in line of sight
      anchor = i - 1;
    }
  }
  waypts.push_back(path.back());

  // Emit smoothed waypoints, subdividing long straight runs so the follower
  // gets intermediate targets to track and re-check for obstacles along.
  const double max_seg = std::max(p_.waypoint_spacing, res);
  int emitted = 0;
  for (size_t k = 1; k < waypts.size(); ++k) {
    const double x0 = cellX(waypts[k - 1] % w), y0 = cellY(waypts[k - 1] / w);
    const double x1 = cellX(waypts[k] % w),     y1 = cellY(waypts[k] / w);
    const double seg = dist2d(x0, y0, x1, y1);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg / max_seg)));
    for (int s = 1; s <= steps; ++s) {
      if (emitted >= p_.max_plan_points) return false;
      const double t = static_cast<double>(s) / steps;
      Waypoint wp = goal;
      // Intermediate points: track tightly and slowly so the follower hugs the
      // corridor. out.back() is overwritten with `goal` below, so the final
      // target keeps the caller's (looser) arrival tol / speed.
      wp.tol   = std::min(goal.tol,   p_.waypoint_tol);
      wp.v_max = std::min(goal.v_max, p_.waypoint_v_max);
      wp.x = x0 + t * (x1 - x0);
      wp.y = y0 + t * (y1 - y0);
      out.push_back(wp);
      ++emitted;
    }
  }
  if (out.empty()) out.push_back(goal);
  out.back() = goal;  // ensure final target is exactly the goal

  // Remember this path so the next replan biases toward the same detour side.
  {
    std::vector<std::pair<double, double>> wp_xy;
    wp_xy.reserve(out.size());
    for (const auto& wp : out) wp_xy.emplace_back(wp.x, wp.y);
    std::lock_guard<std::mutex> lk(mu_);
    last_path_ = std::move(wp_xy);
  }

  ROS_INFO("planner: path cells=%zu turns=%zu waypoints=%zu inflation=%.2f margin=%.2f "
           "start=(%.2f,%.2f) goal=(%.2f,%.2f)",
           path.size(), waypts.size(), out.size(), inflation_radius, xy_margin,
           start_x, start_y, goal.x, goal.y);
  return true;
}

}  // namespace safe_landing
