#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <string>

struct Pt2 {
  double x{0}, y{0};
  int id{-1};          // 原始点ID（用于找原始距离）
};

struct AABB2D {
  double xmin, ymin, xmax, ymax;
};

struct FrontierCluster2D {
  std::vector<Pt2> pts;   // 这里的点就是 Vi（聚类点）
  AABB2D aabb;            // 用于触发更新（与 local 圆相交）
};

struct FeasibleRegion2D {
  std::vector<Pt2> poly;  // 非凸多边形边界（闭合与否都可）
};

static inline double norm2(const Pt2& p) { return std::sqrt(p.x*p.x + p.y*p.y); }
static inline double cross(const Pt2& O, const Pt2& A, const Pt2& B) {
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// 2D convex hull (Andrew monotonic chain). 返回 hull 顶点（逆时针，不重复首尾）
static std::vector<Pt2> convexHull2D(std::vector<Pt2> pts) {
  if (pts.size() <= 2) return pts;
  std::sort(pts.begin(), pts.end(), [](const Pt2& a, const Pt2& b){
    if (a.x != b.x) return a.x < b.x;
    return a.y < b.y;
  });

  std::vector<Pt2> H;
  H.reserve(pts.size() * 2);

  // lower
  for (const auto& p : pts) {
    while (H.size() >= 2 && cross(H[H.size()-2], H[H.size()-1], p) <= 0) H.pop_back();
    H.push_back(p);
  }
  // upper
  size_t lower_size = H.size();
  for (int i = (int)pts.size() - 2; i >= 0; --i) {
    const auto& p = pts[i];
    while (H.size() > lower_size && cross(H[H.size()-2], H[H.size()-1], p) <= 0) H.pop_back();
    H.push_back(p);
  }
  if (!H.empty()) H.pop_back(); // remove duplicated start
  return H;
}

// 2D grid/voxel filter: leaf = 分辨率
static std::vector<Pt2> gridFilter2D(const std::vector<Pt2>& in, double leaf) {
  if (leaf <= 0) return in;
  std::unordered_map<long long, Pt2> cell; // 每格保留一个点
  cell.reserve(in.size());

  auto keyOf = [leaf](double x, double y)->long long {
    long long ix = (long long)std::floor(x / leaf);
    long long iy = (long long)std::floor(y / leaf);
    return (ix << 32) ^ (iy & 0xffffffff);
  };

  for (const auto& p : in) {
    long long k = keyOf(p.x, p.y);
    if (cell.find(k) == cell.end()) cell[k] = p;
  }

  std::vector<Pt2> out;
  out.reserve(cell.size());
  for (auto& kv : cell) out.push_back(kv.second);
  return out;
}

// 简单DBSCAN(可选，用于聚类展示；对主流程不强依赖)
static std::vector<int> dbscan2D(const std::vector<Pt2>& pts, double eps, int minPts) {
  const int n = (int)pts.size();
  std::vector<int> labels(n, 0); // 0=unvisited, -1=noise, >0 clusterId
  int cid = 0;

  auto dist = [](const Pt2& a, const Pt2& b){
    double dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
  };

  for (int i=0;i<n;i++){
    if (labels[i] != 0) continue;
    // region query
    std::vector<int> neighbors;
    neighbors.reserve(64);
    for (int j=0;j<n;j++){
      if (dist(pts[i], pts[j]) <= eps) neighbors.push_back(j);
    }
    if ((int)neighbors.size() < minPts){
      labels[i] = -1;
      continue;
    }
    cid++;
    labels[i] = cid;

    // expand
    for (size_t k=0;k<neighbors.size();k++){
      int p = neighbors[k];
      if (labels[p] == -1) labels[p] = cid;
      if (labels[p] != 0) continue;
      labels[p] = cid;

      std::vector<int> nbr2;
      nbr2.reserve(64);
      for (int j=0;j<n;j++){
        if (dist(pts[p], pts[j]) <= eps) nbr2.push_back(j);
      }
      if ((int)nbr2.size() >= minPts){
        neighbors.insert(neighbors.end(), nbr2.begin(), nbr2.end());
      }
    }
  }
  return labels;
}

static std::vector<std::vector<Pt2>> cluster2D_RadiusCC(
    const std::vector<Pt2>& pts,
    double connect_radius,
    int min_cluster_size = 3)
{
  std::vector<std::vector<Pt2>> clusters;
  const int n = (int)pts.size();
  if (n == 0) return clusters;

  const double r2 = connect_radius * connect_radius;
  std::vector<char> visited(n, 0);

  auto dist2 = [](const Pt2& a, const Pt2& b){
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx*dx + dy*dy;
  };

  for (int i = 0; i < n; ++i) {
    if (visited[i]) continue;

    std::vector<int> q;
    q.reserve(64);
    q.push_back(i);
    visited[i] = 1;

    std::vector<Pt2> cluster;
    cluster.reserve(64);

    for (size_t qi = 0; qi < q.size(); ++qi) {
      int u = q[qi];
      cluster.push_back(pts[u]);

      // 朴素邻域查找：O(n)
      for (int v = 0; v < n; ++v) {
        if (visited[v]) continue;
        if (dist2(pts[u], pts[v]) <= r2) {
          visited[v] = 1;
          q.push_back(v);
        }
      }
    }

    if ((int)cluster.size() >= min_cluster_size)
      clusters.push_back(std::move(cluster));
  }

  return clusters;
}

static void publishClustersPoints(
    const ros::Publisher& pub,
    const std_msgs::Header& header,
    const std::string& ns,
    const std::vector<std::vector<Pt2>>& clusters,
    float r, float g, float b,
    double point_size = 0.06)
{
  int id = 0;
  for (const auto& c : clusters) {
    if (c.empty()) continue;

    visualization_msgs::Marker mk;
    mk.header = header;
    mk.ns = ns;
    mk.id = id++;
    mk.type = visualization_msgs::Marker::POINTS;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = point_size;
    mk.scale.y = point_size;
    mk.color.a = 1.0;
    mk.color.r = r; mk.color.g = g; mk.color.b = b;

    mk.points.reserve(c.size());
    for (const auto& p : c) {
      geometry_msgs::Point gp;
      gp.x = p.x; gp.y = p.y; gp.z = 0.0;
      mk.points.push_back(gp);
    }

    pub.publish(mk);
  }
}

// static std::array<Pt2, 4> aabbVerticesShrink(
//     const std::vector<Pt2>& cluster,
//     double shrink_margin,   // 每条边向内收缩(米)，比如 0.05
//     double min_size = 0.02  // 防止收缩成退化盒
// ) {
//   double xmin =  std::numeric_limits<double>::infinity();
//   double ymin =  std::numeric_limits<double>::infinity();
//   double xmax = -std::numeric_limits<double>::infinity();
//   double ymax = -std::numeric_limits<double>::infinity();

//   for (const auto& p : cluster) {
//     xmin = std::min(xmin, p.x);
//     ymin = std::min(ymin, p.y);
//     xmax = std::max(xmax, p.x);
//     ymax = std::max(ymax, p.y);
//   }

//   // shrink inward
//   xmin += shrink_margin; ymin += shrink_margin;
//   xmax -= shrink_margin; ymax -= shrink_margin;

//   // 防退化：保证宽高至少 min_size
//   if (xmax - xmin < min_size) {
//     double cx = 0.5 * (xmin + xmax);
//     xmin = cx - 0.5 * min_size;
//     xmax = cx + 0.5 * min_size;
//   }
//   if (ymax - ymin < min_size) {
//     double cy = 0.5 * (ymin + ymax);
//     ymin = cy - 0.5 * min_size;
//     ymax = cy + 0.5 * min_size;
//   }

//   // 返回四个顶点：左下、左上、右上、右下（逆时针）
//   return { Pt2{xmin, ymin}, Pt2{xmin, ymax}, Pt2{xmax, ymax}, Pt2{xmax, ymin} };
// }

static std::vector<std::array<Pt2,4>> buildAABBVerticesForClusters(
    const std::vector<std::vector<Pt2>>& clusters,
    int min_cluster_pts = 3
) {
  std::vector<std::array<Pt2,4>> aabbs;
  aabbs.reserve(clusters.size());
  for (const auto& c : clusters) {
    if ((int)c.size() < min_cluster_pts) continue;
    // aabbs.push_back(aabbVerticesShrink(c, shrink_margin));
  }
  return aabbs;
}

static AABB2D computeAABB(const std::vector<Pt2>& pts) {
  AABB2D b;
  b.xmin =  std::numeric_limits<double>::infinity();
  b.ymin =  std::numeric_limits<double>::infinity();
  b.xmax = -std::numeric_limits<double>::infinity();
  b.ymax = -std::numeric_limits<double>::infinity();
  for (const auto& p : pts) {
    b.xmin = std::min(b.xmin, p.x);
    b.ymin = std::min(b.ymin, p.y);
    b.xmax = std::max(b.xmax, p.x);
    b.ymax = std::max(b.ymax, p.y);
  }
  return b;
}

// static AABB2D shrinkAABB(const AABB2D& b, double margin, double min_size=0.02) {
//   AABB2D s = b;
//   s.xmin += margin; s.ymin += margin;
//   s.xmax -= margin; s.ymax -= margin;
//   if (s.xmax - s.xmin < min_size) {
//     double cx = 0.5*(s.xmin+s.xmax);
//     s.xmin = cx - 0.5*min_size; s.xmax = cx + 0.5*min_size;
//   }
//   if (s.ymax - s.ymin < min_size) {
//     double cy = 0.5*(s.ymin+s.ymax);
//     s.ymin = cy - 0.5*min_size; s.ymax = cy + 0.5*min_size;
//   }
//   return s;
// }

static void publishAllAABBs(
    const ros::Publisher& pub,
    const std_msgs::Header& header,
    const std::string& ns,
    const std::vector<std::array<Pt2,4>>& aabbs,
    double line_width = 0.03,
    float r = 1.0f, float g = 1.0f, float b = 0.0f  // 默认黄框，想红框就(1,0,0)
) {
  visualization_msgs::Marker mk;
  mk.header = header;
  mk.ns = ns;
  mk.id = 0;
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = line_width;
  mk.color.a = 1.0;
  mk.color.r = r; mk.color.g = g; mk.color.b = b;

  // 每个 AABB 4条边： (0-1)(1-2)(2-3)(3-0)
  mk.points.reserve(aabbs.size() * 8);

  auto push = [&](const Pt2& p){
    geometry_msgs::Point gp;
    gp.x = p.x; gp.y = p.y; gp.z = 0.0;
    mk.points.push_back(gp);
  };

  for (const auto& v : aabbs) {
    push(v[0]); push(v[1]);
    push(v[1]); push(v[2]);
    push(v[2]); push(v[3]);
    push(v[3]); push(v[0]);
  }

  pub.publish(mk);
}

static bool aabbIntersectsCircle(const AABB2D& b, const Pt2& c, double r) {
  // clamp 圆心到 AABB，算最近点距离
  double x = std::max(b.xmin, std::min(c.x, b.xmax));
  double y = std::max(b.ymin, std::min(c.y, b.ymax));
  double dx = x - c.x, dy = y - c.y;
  return (dx*dx + dy*dy) <= r*r;
}

static double cross_(const Pt2& a, const Pt2& b, const Pt2& c) {
  return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}

static bool onSegment(const Pt2& a, const Pt2& b, const Pt2& p, double eps=1e-9) {
  return std::min(a.x,b.x)-eps <= p.x && p.x <= std::max(a.x,b.x)+eps &&
         std::min(a.y,b.y)-eps <= p.y && p.y <= std::max(a.y,b.y)+eps &&
         std::fabs(cross_(a,b,p)) <= eps;
}

static bool segmentsIntersect(const Pt2& a, const Pt2& b, const Pt2& c, const Pt2& d, double eps=1e-9) {
  double c1 = cross_(a,b,c), c2 = cross_(a,b,d);
  double c3 = cross_(c,d,a), c4 = cross_(c,d,b);

  if ((c1>eps && c2<-eps) || (c1<-eps && c2>eps)) {
    if ((c3>eps && c4<-eps) || (c3<-eps && c4>eps)) return true;
  }
  // collinear / touch
  if (std::fabs(c1) <= eps && onSegment(a,b,c,eps)) return true;
  if (std::fabs(c2) <= eps && onSegment(a,b,d,eps)) return true;
  if (std::fabs(c3) <= eps && onSegment(c,d,a,eps)) return true;
  if (std::fabs(c4) <= eps && onSegment(c,d,b,eps)) return true;
  return false;
}

static bool segmentIntersectsPolygonBoundary(const Pt2& p0, const Pt2& p1,
                                             const std::vector<Pt2>& poly,
                                             double eps=1e-9)
{
  if (poly.size() < 3) return false;
  const int n = (int)poly.size();
  for (int i=0;i<n;i++){
    const Pt2& a = poly[i];
    const Pt2& b = poly[(i+1)%n];
    // 如果交点恰好在 p0 或 p1 上，按你需求可选择“算相交/不算相交”
    if (segmentsIntersect(p0,p1,a,b,eps)) return true;
  }
  return false;
}

static double dist(const Pt2& a, const Pt2& b) {
  double dx=a.x-b.x, dy=a.y-b.y;
  return std::sqrt(dx*dx + dy*dy);
}

// 判断 Vi 是否在“新可行域(2D多边形)”内：用“线段不穿边界”的可见性判据
static bool inNewFeasibleByNoIntersection(const Pt2& Pnow, const Pt2& Vi,
                                         const FeasibleRegion2D& new_region)
{
  // 若 ViPnow 与边界相交 => Vi 在域外（或不可直视）
  return !segmentIntersectsPolygonBoundary(Pnow, Vi, new_region.poly);
}

static void updateExistingFrontiersByNewRegion(
    std::vector<FrontierCluster2D>& frontiers,  // in/out
    const Pt2& Pnow,
    double Rlocal,
    const FeasibleRegion2D& new_region)            
{
  std::vector<FrontierCluster2D> updated;
  updated.reserve(frontiers.size());

  for (auto& F : frontiers) {
    // 1) 触发更新：旧 frontier 的 AABB 是否与新 local sphere(圆)相交
    if (!aabbIntersectsCircle(F.aabb, Pnow, Rlocal)) {
      // 不相交 => 不需要更新
      updated.push_back(F);
      continue;
    }

    // 2) 相交 => 对 cluster 内每个点 Vi 做 Type A/B/C 判定并删除
    std::vector<Pt2> kept;
    kept.reserve(F.pts.size());

    for (const auto& Vi : F.pts) {
      double Di = dist(Vi, Pnow);

      if (Di > Rlocal) {
        // Type A: out of local sphere -> keep
        kept.push_back(Vi);
        continue;
      }

      // Di < Rlocal: 再判定是否在新可行域
      bool in_new_feasible = inNewFeasibleByNoIntersection(Pnow, Vi, new_region);

      if (in_new_feasible) {
        // Type B: in known area -> delete (do nothing)
      } else {
        // Type C: inside sphere but not in feasible -> keep
        kept.push_back(Vi);
      }
    }

    // 3) 删除后若 cluster 还有点 => 重算 AABB（并可 shrink）
    if (!kept.empty()) {
      FrontierCluster2D G;
      G.pts = std::move(kept);
      G.aabb = computeAABB(G.pts);
      // if (aabb_shrink_margin > 0.0) G.aabb = shrinkAABB(G.aabb, aabb_shrink_margin);
      updated.push_back(std::move(G));
    }
    // 若空了 => frontier cluster 直接消失
  }

  frontiers.swap(updated);
}

static bool pointInPolygonRayCasting(const Pt2& p, const std::vector<Pt2>& poly) {
  // 射线法（不要求凸）
  bool inside = false;
  int n = (int)poly.size();
  for (int i=0, j=n-1; i<n; j=i++) {
    const Pt2& a = poly[i];
    const Pt2& b = poly[j];
    bool intersect = ((a.y > p.y) != (b.y > p.y)) &&
                     (p.x < (b.x-a.x) * (p.y-a.y) / (b.y-a.y + 1e-12) + a.x);
    if (intersect) inside = !inside;
  }
  return inside;
}

static bool inAnyPreviousRegion(const Pt2& v, const std::vector<FeasibleRegion2D>& prev_regions) {
  for (const auto& R : prev_regions) {
    if (R.poly.size() >= 3 && pointInPolygonRayCasting(v, R.poly)) return true;
  }
  return false;
}

static void filterCandidateFrontiersByPreviousRegions(
    std::vector<FrontierCluster2D>& candidates,              // in/out
    const std::vector<FeasibleRegion2D>& prev_regions)
{
  std::vector<FrontierCluster2D> out;
  out.reserve(candidates.size());

  for (auto& C : candidates) {
    std::vector<Pt2> kept;
    kept.reserve(C.pts.size());

    for (const auto& v : C.pts) {
      if (inAnyPreviousRegion(v, prev_regions)) {
        // Type A: in explored regions -> delete
      } else {
        // Type B: unknown -> keep
        kept.push_back(v);
      }
    }

    if (!kept.empty()) {
      FrontierCluster2D D;
      D.pts = std::move(kept);
      D.aabb = computeAABB(D.pts);
      // if (aabb_shrink_margin > 0.0) D.aabb = shrinkAABB(D.aabb, aabb_shrink_margin);
      out.push_back(std::move(D));
    }
  }

  candidates.swap(out);
}

static void updateFrontiersRealtime(
    std::vector<FrontierCluster2D>& global_frontiers,      // in/out
    std::vector<FeasibleRegion2D>&  prev_regions,          // in/out
    const Pt2& Pnow,
    double Rlocal,
    const FeasibleRegion2D& new_region,
    std::vector<FrontierCluster2D>& candidate_frontiers   // in/out (will be filtered)
    )
{
  // ---------- Step 1: update existing frontiers by new region ----------
  updateExistingFrontiersByNewRegion(
      global_frontiers,
      Pnow, Rlocal,
      new_region);

  // ---------- Step 2: filter candidate frontiers by previous regions ----------
  filterCandidateFrontiersByPreviousRegions(
      candidate_frontiers,
      prev_regions);

  // ---------- Step 3: merge candidates into global ----------
  // 最简单：直接 append（后续如需融合相近 cluster，可再加 merge）
  for (auto& c : candidate_frontiers) {
    global_frontiers.push_back(std::move(c));
  }
  candidate_frontiers.clear();

  // ---------- Step 4: store new feasible region into explored set ----------
  prev_regions.push_back(new_region);

  // 可选：限制 prev_regions 数量避免无限增长（例如只保留最近 N 个）
  // if (prev_regions.size() > 200) prev_regions.erase(prev_regions.begin(), prev_regions.begin() + 50);
}

class ScanStarRegionNode {
public:
  ScanStarRegionNode() : tf_listener_(tf_buffer_) {
    ros::NodeHandle pnh("~");
    pnh.param("namespace", namespace_, std::string(""));
    pnh.param("scan_topic", scan_topic_, std::string("/scan"));
    pnh.param("fixed_frame", fixed_frame_, std::string("map"));
    pnh.param("base_frame", base_frame_, std::string("/base_link"));
    pnh.param("use_tf_center", use_tf_center_, true);

    pnh.param("Rlocal", Rlocal_, 8.0);
    pnh.param("voxel_leaf", voxel_leaf_, 0.05);

    pnh.param("cluster_enable", cluster_enable_, false);
    pnh.param("cluster_eps", cluster_eps_, 0.25);
    pnh.param("cluster_min_pts", cluster_min_pts_, 8);

    pnh.param("marker_ns", marker_ns_, std::string("star_region"));
    pnh.param("line_width", line_width_, 0.05);

    sub_ = nh_.subscribe(scan_topic_, 1, &ScanStarRegionNode::cbScan, this);
    pub_region_ = nh_.advertise<visualization_msgs::Marker>("star_region_boundary", 1);
    pub_points_ = nh_.advertise<visualization_msgs::Marker>("star_region_points", 1);
    pub_clusters_ = nh_.advertise<visualization_msgs::Marker>("star_region_clusters", 1);
    pub_aabb_ = nh_.advertise<visualization_msgs::Marker>("star_region_aabbs", 1);
    ROS_INFO("[scan_star_region] listening: %s", scan_topic_.c_str());
  }

private:
  void cbScan(const sensor_msgs::LaserScanConstPtr& msg) {
    // 1) UAV中心（2D）：默认 base_frame 原点；也可用TF取 map下位置
    double cx=0.0, cy=0.0, cyaw=0.0;
    if (use_tf_center_) {
      try {
        geometry_msgs::TransformStamped T =
          tf_buffer_.lookupTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
        cx = T.transform.translation.x;
        cy = T.transform.translation.y;
        cyaw = tf2::getYaw(T.transform.rotation);
      } catch (...) {
        // TF拿不到就退化到scan frame原点
        cx = 0; cy = 0; cyaw = 0;
      }
    }

    // 2) 从scan构造 occupied/free 点，并对超出Rlocal的点做投影
    std::vector<Pt2> occ_raw, free_raw;
    occ_raw.reserve(msg->ranges.size());
    free_raw.reserve(msg->ranges.size());

    std::vector<double> original_dist; // 用于回投影（每个点对应的距离）
    std::vector<Pt2> all_projected_to_unit; // unit circle点（保存id）
    original_dist.reserve(msg->ranges.size()*2);
    all_projected_to_unit.reserve(msg->ranges.size()*2);

    auto addPoint = [&](bool is_occ, double x, double y, double dist_to_center) {
      int id = (int)original_dist.size();
      original_dist.push_back(dist_to_center);

      // 投影到单位圆（2D）：dir = (x-c)/(|| ||)
      double dx = x - cx, dy = y - cy;
      double d = std::sqrt(dx*dx + dy*dy);
      if (d < 1e-9) return;

      Pt2 u;
      u.x = dx / d; // unit circle centered at origin (relative)
      u.y = dy / d;
      u.id = id;
      all_projected_to_unit.push_back(u);

      Pt2 p; p.x=x; p.y=y; p.id=id;
      if (is_occ) occ_raw.push_back(p);
      else        free_raw.push_back(p);
    };

    const double angle_min = msg->angle_min;
    const double angle_inc = msg->angle_increment;
    const double rmin = msg->range_min;
    const double rmax = msg->range_max;

    for (size_t i=0;i<msg->ranges.size();i++){
      double a = angle_min + (double)i * angle_inc;
      double c = std::cos(a), s = std::sin(a);

      double r = msg->ranges[i];
      bool finite = std::isfinite(r);

      if (!finite) {
        // 没有回波 => 认为该方向自由到Rlocal
        double rf = Rlocal_;
        double x = cx + rf*c, y = cy + rf*s;
        addPoint(false, x, y, rf);
        continue;
      }

      // clamp到合法范围
      if (r < rmin) r = rmin;
      if (r > rmax) r = rmax;

      if (r <= Rlocal_) {
        // 在局部圆内：作为occupied（障碍点）
        double x = cx + r*c, y = cy + r*s;
        addPoint(true, x, y, r);
      } else {
        // 超出局部圆：投影到局部圆 => free点 (公式(2)在2D的特例)
        double rf = Rlocal_;
        double x = cx + rf*c, y = cy + rf*s;
        addPoint(false, x, y, rf);
      }
    }

    // 3) voxel/grid filter（对 occ/free 各自滤波，保持形状+均匀密度）
    auto occ = gridFilter2D(occ_raw, voxel_leaf_);
    auto fre = gridFilter2D(free_raw, voxel_leaf_);

    // 同时也要对 unit circle 点做滤波（避免凸包被点爆）
    auto unit_pts = gridFilter2D(all_projected_to_unit, 0.002); // unit circle上用更小leaf

    if (unit_pts.size() < 3) {
      publishEmpty(msg);
      return;
    }

    // 4) 可选聚类（用于看不同障碍/自由面片）
    // auto labels = dbscan2D(occ, cluster_eps_, cluster_min_pts_);
    // publishClusters(msg, labels, occ, cx, cy);
    double connect_radius = 3.0 * voxel_leaf_;
    auto occ_clusters = cluster2D_RadiusCC(occ, connect_radius, 5);
    auto fre_clusters = cluster2D_RadiusCC(fre, connect_radius, 10);
    publishClustersPoints(pub_clusters_, msg->header, "occ_clusters", occ_clusters, 1.0, 0.0, 0.0, 0.06);
    publishClustersPoints(pub_clusters_, msg->header, "free_clusters", fre_clusters, 0.0, 1.0, 0.0, 0.06);

    // 5) 在单位圆上做凸包（QuickHull思想；2D用Andrew更稳）
    auto hull_unit = convexHull2D(unit_pts);
    if (hull_unit.size() < 3) {
      publishEmpty(msg);
      return;
    }

    // 6) 根据原始距离，把凸包顶点从单位圆反投影回“局部圆/局部空间”
    //    反投影：P = center + dir * dist(id)
    std::vector<Pt2> poly;
    poly.reserve(hull_unit.size()+1);

    for (const auto& u : hull_unit) {
      int id = u.id;
      if (id < 0 || id >= (int)original_dist.size()) continue;
      double d = original_dist[id];
      Pt2 gp;
      gp.x = cx + u.x * d;
      gp.y = cy + u.y * d;
      poly.push_back(gp);
    }
    if (poly.size() < 3) {
      publishEmpty(msg);
      return;
    }
    // close polygon
    poly.push_back(poly.front());

    // 2) 生成 new_region（你已有模块）
    // FeasibleRegion2D new_region = buildNewLocalFeasibleRegion(...);
    FeasibleRegion2D new_region;
    new_region.poly = poly;

    // 3) 生成 candidate frontier clusters（你已有模块）
    // std::vector<FrontierCluster2D> candidate_frontiers = buildCandidateFrontiers(...);
    // double shrink_margin = 0.05; // 例如 5cm，或 1.0 * voxel_leaf_
    auto aabb_vertices = buildAABBVerticesForClusters(fre_clusters);
    std::vector<FrontierCluster2D> candidate_frontiers;
    // 注意：candidate_frontiers 每个 cluster 至少要填 pts + 初始 aabb
    for (auto& c : fre_clusters) {
      FrontierCluster2D frontier_c;
      auto aabb = computeAABB(c);
      // 如果你想 aabb 本身略小：
      // c.aabb = shrinkAABB(c.aabb, aabb_shrink_margin);
      frontier_c.aabb = aabb;
      frontier_c.pts = c;
      candidate_frontiers.push_back(frontier_c);
    }

    // 4) 实时更新（核心 loop）
    Pt2 Pnow;
    Pnow.x = cx;
    Pnow.y = cy;
    Pnow.id = 0;
    updateFrontiersRealtime(global_frontiers_,
                            prev_regions_,
                            Pnow,
                            Rlocal_,
                            new_region,
                            candidate_frontiers);

    // 5) 可视化（你已有 publish 函数就调用）
    // publishFrontierPoints(global_frontiers_);
    // publishAllAABBs(global_frontiers_);
    // // 可视化（你需要一个 Publisher：ros::Publisher pub_aabb_; 发布 Marker）
    // publishAllAABBs(pub_aabb_, msg->header, "occ_aabbs", aabb_vertices, 0.03, 1.0f, 0.0f, 0.0f); // 红框
    // publishFeasibleRegion(new_region);
    // 7) 可视化：最终非凸边界（LINE_STRIP） + 原始点云（POINTS）
    // publishBoundary(msg, poly);
    publishPoints(msg, occ, fre, cx, cy);
  }

  void publishEmpty(const sensor_msgs::LaserScanConstPtr& msg) {
    visualization_msgs::Marker mk;
    mk.header = msg->header;
    mk.ns = marker_ns_;
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::DELETE;
    pub_region_.publish(mk);
  }

  void publishBoundary(const sensor_msgs::LaserScanConstPtr& msg,
                       const std::vector<geometry_msgs::Point>& poly) {
    visualization_msgs::Marker mk;
    mk.header = msg->header;
    mk.ns = marker_ns_;
    mk.id = 0;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = line_width_;
    mk.color.a = 1.0;
    mk.color.r = 1.0; // 你可改色
    mk.color.g = 0.2;
    mk.color.b = 0.2;
    mk.points = poly;
    pub_region_.publish(mk);
  }

  void publishPoints(const sensor_msgs::LaserScanConstPtr& msg,
                     const std::vector<Pt2>& occ,
                     const std::vector<Pt2>& fre,
                     double cx, double cy) {
    visualization_msgs::Marker mk;
    mk.header = msg->header;
    mk.ns = marker_ns_;
    mk.id = 1;
    mk.type = visualization_msgs::Marker::POINTS;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.color.a = 1.0;
    mk.color.r = 0.2;
    mk.color.g = 0.8;
    mk.color.b = 1.0;

    mk.points.clear();
    mk.points.reserve(occ.size() + fre.size());
    for (auto& p: occ) { geometry_msgs::Point gp; gp.x=p.x; gp.y=p.y; gp.z=0; mk.points.push_back(gp); }
    for (auto& p: fre) { geometry_msgs::Point gp; gp.x=p.x; gp.y=p.y; gp.z=0; mk.points.push_back(gp); }
    pub_points_.publish(mk);
  }

  void publishClusters(const sensor_msgs::LaserScanConstPtr& msg,
                       const std::vector<int>& labels,
                       const std::vector<Pt2>& pts,
                       double cx, double cy) {
    visualization_msgs::Marker mk;
    mk.header = msg->header;
    mk.ns = marker_ns_;
    mk.id = 2;
    mk.type = visualization_msgs::Marker::POINTS;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.06;
    mk.scale.y = 0.06;
    mk.color.a = 1.0;
    mk.color.r = 0.9;
    mk.color.g = 0.9;
    mk.color.b = 0.2;

    mk.points.clear();
    for (size_t i=0;i<pts.size() && i<labels.size();i++){
      if (labels[i] <= 0) continue;
      geometry_msgs::Point gp; gp.x=pts[i].x; gp.y=pts[i].y; gp.z=0;
      mk.points.push_back(gp);
    }
    pub_clusters_.publish(mk);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_region_, pub_points_, pub_clusters_, pub_aabb_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string namespace_;
  std::string scan_topic_;
  std::string fixed_frame_, base_frame_;
  bool use_tf_center_{true};

  double Rlocal_{8.0};
  double voxel_leaf_{0.05};

  bool cluster_enable_{false};
  double cluster_eps_{0.25};
  int cluster_min_pts_{8};

  std::string marker_ns_;
  double line_width_{0.05};

  std::vector<FrontierCluster2D> global_frontiers_;     
  std::vector<FeasibleRegion2D>  prev_regions_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_star_region_node");
  ScanStarRegionNode node;
  ros::spin();
  return 0;
}
