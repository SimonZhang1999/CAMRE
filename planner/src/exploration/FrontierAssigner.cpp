#include "FrontierAssigner.hpp"
#include "hungarian.hpp"

#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <limits>
#include <set>
#include <sstream>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

// ============================================================
// Small local helpers
// ============================================================

namespace {

inline FrontierAssigner::Vec2 NaN2() {
    return {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
}

inline double clamp01(double x) {
    if (x < -1.0) return -1.0;
    if (x >  1.0) return  1.0;
    return x;
}

inline double yawOfPose(const geometry_msgs::Pose& p) {
    const auto& q = p.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

inline std::pair<double,double> dirUnitOrYaw(const geometry_msgs::Pose& pose, const FrontierAssigner::Vec2& tgt) {
    const double px = pose.position.x;
    const double py = pose.position.y;
    double vx = tgt.first  - px;
    double vy = tgt.second - py;
    double n  = std::hypot(vx, vy);
    if (std::isfinite(n) && n > 0.5) return {vx/n, vy/n};
    const double yaw = yawOfPose(pose);
    return {std::cos(yaw), std::sin(yaw)};
}

} // namespace

// ============================================================
// FrontierAssigner ctor
// ============================================================

FrontierAssigner::FrontierAssigner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
{
    scp_ = std::make_shared<star_convex_handle::StarConvexHandler>(nh_);

    pub_assigned_targets_marker_ = nh_.advertise<visualization_msgs::Marker>("assigned_targets_marker", 1);
    pub_temp_targets_marker_     = nh_.advertise<visualization_msgs::Marker>("temp_targets_marker", 1);
    pub_record_traj_info_ = nh.advertise<std_msgs::Float64MultiArray>("/record_traj_topic", 10);
    
    pnh.param<std::string>("global_frame", global_frame_, std::string("map"));
    pnh.param("target_marker_scale", target_marker_scale_, 0.35);
    pnh.param("frontier_robot_min_dist", frontier_robot_min_dist_, 1.5);
    pnh.param("inter_robot_threshold", inter_robot_threshold_, 1.8);
    pnh.param("cluster_window", cluster_window_, 10.0);
    pnh.param("cluster_radius", cluster_radius_, 3.0);
    pnh.param("frontier_merge_eps", frontier_merge_eps_, 0.35);
    pnh.param("goal_dedup_eps", goal_dedup_eps_, 0.45);
    pnh.param("stuck_retry_merge_eps", stuck_retry_merge_eps_, 0.45);
    pnh.param("perm_block_merge_eps", perm_block_merge_eps_, 0.3);
    pnh.param("jump_guard_dist_th", jump_guard_dist_th_, 20.0);
    pnh.param("jump_guard_yaw_th", jump_guard_yaw_th_, 90.0);
    pnh.param("jump_guard_max_ct", jump_guard_max_ct_, 3);
    pnh.param("stuck_move_eps", stuck_move_eps_, 0.1);
    pnh.param("stuck_prog_eps", stuck_prog_eps_, 0.1);
    pnh.param("stuck_cycles", stuck_cycles_, 25);
    pnh.param("block_merge_eps", block_merge_eps_, 0.3);
    pnh.param("robot_forbidden_merge_eps", robot_forbidden_merge_eps_, 0.45);
    pnh.param("anchor_lock_match_eps", anchor_lock_match_eps_, 0.6);
    pnh.param("anchor_hist_drop_eps", anchor_hist_drop_eps_, 2.0);
    pnh.param("anchor_hist_merge_eps", anchor_hist_merge_eps_, 0.3);
    pnh.param("anchor_hist_max_size", anchor_hist_max_size_, 40000);

    pnh.param("marker_lifetime", marker_lifetime_, 0.3);
    pnh.param("num_robots", num_robot_, 0);
    pnh.param("min_dist", MIN_TGT_DIST, 0.0);
    pnh.param("max_dist", MAX_TGT_DIST, 0.0);
    pnh.param("max_heading_angle_deg", max_heading_angle_deg_, 60.0);
    pnh.param("reach_radius", reach_radius_, 0.6);
    pnh.param("ref_inertia_alpha", ref_inertia_alpha_, 0.0);
    pnh.param("ref_inertia_max_step", ref_inertia_max_step_, 60.0);
    pnh.param("ref_inertia_min_step", ref_inertia_min_step_, 0.6);
    pnh.param("occ_threshold", occ_threshold_, 50);
    pnh.param("treat_unknown_as_occupied", treat_unknown_as_occupied_, false);
    pnh.param("los_inflate_cells", los_inflate_cells_, 1);

    pnh.param("anchor_pause_K", anchor_pause_K_, 3);
    pnh.param("all_orange_relax_M", all_orange_relax_M_, 5);
    pnh.param("los_penalty", los_penalty_, 10.0);
    if (num_robot_ <= 0) {
        ROS_FATAL("FrontierAssigner: param '~num_robot' must be > 0");
        throw std::runtime_error("invalid num_robot");
    }

    pub_targets_type_.resize(num_robot_);
    for (int i = 0; i < num_robot_; i++) {
        pub_targets_type_[i] = nh_.advertise<std_msgs::Bool>("/robot_"+std::to_string(i+1)+"/target_type", 1);
    }

    if (!pnh.getParam("frontier_topics", frontier_topics_)) {
        frontier_topics_.resize(num_robot_);
        for (int i = 0; i < num_robot_; ++i) {
            frontier_topics_[i] = "/robot_" + std::to_string(i + 1) + "/centroids_points";
        }
        ROS_WARN("~frontier_topics not set; auto-using /robot_i/centroids_points");
    } else {
        if ((int)frontier_topics_.size() != num_robot_) {
            ROS_FATAL("frontier_topics size (%zu) != num_robot (%d)", frontier_topics_.size(), num_robot_);
            throw std::runtime_error("frontier_topics size mismatch");
        }
    }

    // Allocate shared vectors
    poses_.resize(num_robot_);
    record_traj_info_.resize(num_robot_);
    robot_neighbors_.resize(num_robot_);
    frontiers_by_robot_.resize(num_robot_);

    has_pose_.assign(num_robot_, false);
    has_frontiers_.assign(num_robot_, false);
    has_convexhulls_.assign(num_robot_, false);
    has_pc_.assign(num_robot_, false);
    has_emergencystop_.assign(num_robot_, false);

    emergencystop_robots_.assign(num_robot_, false);

    robots_convexhull_.resize(num_robot_);
    robots_neighbor_pc_.resize(num_robot_);
    jump_guard_ct_.assign(num_robot_, 0);

    goal_pubs_.resize(num_robot_);
    pose_subs_.resize(num_robot_);
    frontier_subs_.resize(num_robot_);
    subs_vertices_convexhull_.resize(num_robot_);
    emergency_stop_subs_.resize(num_robot_);
    PC_subs_.resize(num_robot_);

    last_targets_xy_.assign(num_robot_, NaN2());
    has_target_.assign(num_robot_, false);
    last_is_temp_.assign(num_robot_, true); // default ORANGE

    last_pose_xy_.assign(num_robot_, NaN2());
    last_dist_to_goal_.assign(num_robot_, std::numeric_limits<double>::infinity());
    stuck_ct_.assign(num_robot_, 0);

    // publishing cache
    last_published_goal_.assign(num_robot_, NaN2());
    has_published_goal_.assign(num_robot_, false);
    last_published_is_temp_.assign(num_robot_, true);

    // GREEN memory
    last_green_targets_xy_.assign(num_robot_, NaN2());
    has_green_target_.assign(num_robot_, false);

    should_publish_count_ = 0;
    robot_forbidden_targets_.assign(num_robot_, {});
    robot_forbidden_merge_eps_ = goal_dedup_eps_; // 直接复用你的去重半径
    anchor_lock_target_= NaN2();
    // Subscribers
    edges_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/mst_graph/edges", 1,
                                                           &FrontierAssigner::edgesCb, this);
    sub_planning_map_ = nh.subscribe("/map", 1, &FrontierAssigner::planningMapCb, this);

    for (int i = 0; i < num_robot_; ++i) {
        pose_subs_[i] = nh_.subscribe<nav_msgs::Odometry>(
            "/robot_" + std::to_string(i + 1) + "/RosAria/odom", 1,
            boost::bind(&FrontierAssigner::poseCb, this, _1, i));

        frontier_subs_[i] = nh_.subscribe<cure_planner::PointArray>(
            frontier_topics_[i], 1,
            boost::bind(&FrontierAssigner::frontiersCb, this, _1, i));

        emergency_stop_subs_[i] = nh_.subscribe<std_msgs::Bool>(
            "/robot_" + std::to_string(i + 1) + "/planner/solving_failed", 1,
            boost::bind(&FrontierAssigner::emergencystopCb, this, _1, i));

        subs_vertices_convexhull_[i] = nh_.subscribe<std_msgs::Float32MultiArray>(
            "/robot_" + std::to_string(i + 1) + "/vertices_convexhull", 1,
            boost::bind(&FrontierAssigner::convexhullsCb, this, _1, i));

        PC_subs_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/robot_" + std::to_string(i + 1) + "/NeighborPointcloud", 1,
            boost::bind(&FrontierAssigner::neiPcCb, this, _1, i));

        const std::string goal_topic = "/robot_" + std::to_string(i + 1) + "/move_base_simple/goal";
        goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    }
    pub_clustered_frontiers_marker_ =
        nh_.advertise<visualization_msgs::Marker>("clustered_frontiers_marker", 1);
    ROS_INFO("FrontierAssigner started. num_robot=%d, global_frame=%s",
             num_robot_, global_frame_.c_str());
}

// ============================================================
// Callbacks
// ============================================================

void FrontierAssigner::poseCb(const nav_msgs::Odometry::ConstPtr& msg, int idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    try {
        has_pose_[idx] = true;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp, ros::Duration(0.1));
        const double x = transformStamped.transform.translation.x;
        const double y = transformStamped.transform.translation.y;
        const double yaw = tf::getYaw(transformStamped.transform.rotation);
        
        geometry_msgs::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = 0.0;
        p.orientation = tf::createQuaternionMsgFromYaw(yaw);
        poses_[idx] = p;
        record_traj_info_[idx] = {x, y, yaw, msg->twist.twist.linear.x, msg->twist.twist.angular.z};
    } catch (const tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    }
}

void FrontierAssigner::frontiersCb(const cure_planner::PointArrayConstPtr& msg, int robot_idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<Vec2> list;
    list.reserve(msg->points.size());
    for (const auto& p : msg->points) list.emplace_back(p.x, p.y);
    frontiers_by_robot_[robot_idx] = std::move(list);
    has_frontiers_[robot_idx] = true;
}

void FrontierAssigner::convexhullsCb(const std_msgs::Float32MultiArray::ConstPtr& msg, int idx) {
    std::lock_guard<std::mutex> lock(mtx_);

    for (bool ok : has_pose_) if (!ok) return;

    const auto& data = msg->data;
    if (data.size() % 2 != 0) {
        ROS_WARN_THROTTLE(1.0, "convexhullsCb: data size not even, ignore.");
        return;
    }

    std::vector<Eigen::Vector2d> tmp;
    tmp.reserve(data.size() / 2);
    for (size_t i = 0; i < data.size(); i += 2) {
        tmp.emplace_back(static_cast<double>(data[i]),
                         static_cast<double>(data[i + 1]));
    }

    robots_convexhull_[idx] = std::move(tmp);
    has_convexhulls_[idx] = true;
}

void FrontierAssigner::neiPcCb(const sensor_msgs::PointCloud2ConstPtr& msg, int robot_idx) {
    std::lock_guard<std::mutex> lock(mtx_);

    std::vector<Eigen::Vector2f> tmp;
    tmp.reserve((size_t)msg->width * (size_t)msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");

    for (; it_x != it_x.end(); ++it_x, ++it_y) tmp.emplace_back(*it_x, *it_y);

    robots_neighbor_pc_[robot_idx] = std::move(tmp);
    has_pc_[robot_idx] = true;
}

void FrontierAssigner::edgesCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    have_graph_ = true;
    robot_neighbors_.assign(num_robot_, std::vector<int>{});

    const size_t n = msg->data.size();

    const bool is_triple = (n % 3 == 0);
    const bool is_pair   = (!is_triple && (n % 2 == 0));

    size_t step = 0;
    if (is_triple) step = 3;
    else if (is_pair) step = 2;
    else {
        ROS_WARN_THROTTLE(1.0, "edgesCb: size %zu not divisible by 2 or 3, ignore", n);
        return;
    }

    for (size_t i = 0; i + 1 < n; i += step) {
        int u = static_cast<int>(std::lround(msg->data[i])) - 1;
        int v = static_cast<int>(std::lround(msg->data[i + 1])) - 1;

        if (u < 0 || u >= num_robot_ || v < 0 || v >= num_robot_) {
            ROS_WARN_THROTTLE(1.0, "edgesCb: invalid edge (%d,%d) for num_robot=%d, skip", u + 1, v + 1, num_robot_);
            continue;
        }
        if (u == v) continue;

        robot_neighbors_[u].push_back(v);
        robot_neighbors_[v].push_back(u);
    }

    for (auto& neis : robot_neighbors_) {
        std::sort(neis.begin(), neis.end());
        neis.erase(std::unique(neis.begin(), neis.end()), neis.end());
    }
}

void FrontierAssigner::emergencystopCb(const std_msgs::Bool::ConstPtr& msg, int robot_idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    emergencystop_robots_[robot_idx] = msg->data;
    has_emergencystop_[robot_idx] = true;
}

void FrontierAssigner::planningMapCb(const nav_msgs::OccupancyGridConstPtr& msg) {
    std::lock_guard<std::mutex> lk(map_mtx_);
    map_ = *msg;
    has_map_ = true;
}

// ============================================================
// Ready checks
// ============================================================

bool FrontierAssigner::allPosesReady() const {
    std::lock_guard<std::mutex> lock(mtx_);
    for (bool ok : has_pose_) if (!ok) return false;
    return true;
}

bool FrontierAssigner::allConvexHullsReady() const {
    std::lock_guard<std::mutex> lock(mtx_);
    for (bool ok : has_convexhulls_) if (!ok) return false;
    return true;
}

bool FrontierAssigner::allEmergencyStopReady() const {
    std::lock_guard<std::mutex> lock(mtx_);
    for (bool ok : has_emergencystop_) if (!ok) return false;
    return true;
}

bool FrontierAssigner::allFrontiersReady() const {
    std::lock_guard<std::mutex> lock(mtx_);
    for (bool ok : has_frontiers_) if (!ok) return false;
    return true;
}

bool FrontierAssigner::allPCReady() const {
    std::lock_guard<std::mutex> lock(mtx_);
    for (bool ok : has_pc_) if (!ok) return false;
    return true;
}

// ============================================================
// Snapshot
// ============================================================

FrontierAssigner::Snapshot FrontierAssigner::getSnapshotLocked() const {
    // std::lock_guard<std::mutex> lock(mtx_);

    Snapshot s;
    s.poses = poses_;
    s.frontiers_by_robot = frontiers_by_robot_;
    s.neighbors = robot_neighbors_;
    s.hulls = robots_convexhull_;
    s.neighbor_pc = robots_neighbor_pc_;

    s.has_target = has_target_;
    s.last_is_temp = last_is_temp_;
    s.last_targets = last_targets_xy_;
    s.emerg_stop = emergencystop_robots_;

    s.has_pose = has_pose_;
    s.has_hull = has_convexhulls_;
    s.has_pc = has_pc_;
    s.has_frontier = has_frontiers_;

    s.have_graph = have_graph_;

    s.last_green_targets = last_green_targets_xy_;
    s.has_green_target   = has_green_target_;
    return s;
}

// ============================================================
// Small helpers
// ============================================================

double FrontierAssigner::yawFromTo(const geometry_msgs::Pose& from, const Vec2& to) {
    return std::atan2(to.second - from.position.y, to.first - from.position.x);
}

void FrontierAssigner::yawToQuaternion(double yaw, double& qx, double& qy, double& qz, double& qw) {
    const double half = yaw * 0.5;
    qx = 0.0;
    qy = 0.0;
    qz = std::sin(half);
    qw = std::cos(half);
}

void FrontierAssigner::publishMoveBaseSimpleGoal(int robot_idx, const Vec2& target_xy,
                                                 const geometry_msgs::Pose& robot_xy) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = target_xy.first;
    goal.pose.position.y = target_xy.second;
    goal.pose.position.z = 0.0;

    const double yaw = yawFromTo(robot_xy, target_xy);
    double qx, qy, qz, qw;
    yawToQuaternion(yaw, qx, qy, qz, qw);
    goal.pose.orientation.x = qx;
    goal.pose.orientation.y = qy;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;

    goal_pubs_[robot_idx].publish(goal);
}
void FrontierAssigner::pruneAnchorHistoryIfNeeded() {
    if (anchor_hist_max_size_ <= 0) return;
    if ((int)anchor_green_history_.size() <= anchor_hist_max_size_) return;

    // simple: drop oldest
    const int extra = (int)anchor_green_history_.size() - anchor_hist_max_size_;
    anchor_green_history_.erase(anchor_green_history_.begin(),
                                anchor_green_history_.begin() + extra);
}

void FrontierAssigner::rememberAnchorTarget(const Vec2& p) {
    if (!isFiniteVec2(p)) return;

    const double eps2 = anchor_hist_merge_eps_ * anchor_hist_merge_eps_;
    for (const auto& h : anchor_green_history_) {
        const double dx = p.first  - h.first;
        const double dy = p.second - h.second;
        if (dx*dx + dy*dy <= eps2) return; // already stored
    }
    anchor_green_history_.push_back(p);
    pruneAnchorHistoryIfNeeded();
}

bool FrontierAssigner::nearAnyAnchorHistory(const Vec2& p) const {
    if (!isFiniteVec2(p)) return true;
    if (anchor_green_history_.empty()) return false;

    const double r2 = anchor_hist_drop_eps_ * anchor_hist_drop_eps_;
    for (const auto& h : anchor_green_history_) {
        const double dx = p.first  - h.first;
        const double dy = p.second - h.second;
        if (dx*dx + dy*dy <= r2) return true;
    }
    return false;
}
// ============================================================
// Legacy helpers (kept)
// ============================================================

bool FrontierAssigner::FrontiersFeasibility(const Vec2& frontier) const {
    // std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& pose : poses_) {
        if (std::hypot(pose.position.x - frontier.first,
                       pose.position.y - frontier.second) < 1.0) {
            return false;
        }
    }
    return true;
}

std::vector<FrontierAssigner::Vec2> FrontierAssigner::mergeAllFrontiersLocked() const {
    std::vector<Vec2> merged;
    for (int i = 0; i < num_robot_; ++i) {
        for (const auto& fi : frontiers_by_robot_[i]) {
            if (!FrontiersFeasibility(fi)) continue;
            merged.push_back(fi);
        }
    }
    return merged;
}

std::vector<FrontierAssigner::Vec2> FrontierAssigner::getMergedFrontiers() const {
    // std::lock_guard<std::mutex> lock(mtx_);
    return mergeAllFrontiersLocked();
}

double FrontierAssigner::distanceToNearestObstacle(const nav_msgs::OccupancyGrid& map,
                                                   double wx, double wy,
                                                   double search_radius_m) const
{
    int cx, cy;
    if (!worldToGrid(map, wx, wy, cx, cy)) {
        return std::numeric_limits<double>::infinity();
    }

    const double res = map.info.resolution;
    const int r = static_cast<int>(std::ceil(search_radius_m / res));
    constexpr int obstacle_threshold = 50;

    double best = std::numeric_limits<double>::infinity();
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            const int gx = cx + dx;
            const int gy = cy + dy;
            if (gx < 0 || gy < 0 || gx >= (int)map.info.width || gy >= (int)map.info.height) continue;

            const int occ = gridValueAt(map, gx, gy);
            if (occ < obstacle_threshold) continue;

            const double d = std::hypot(dx * res, dy * res);
            if (d < best) best = d;
        }
    }
    return best;
}

bool FrontierAssigner::CheckExistingTargets(const int& ego_idx, const double& x, const double& y,
                                            const std::vector<Vec2>& target_set) {
    for (int i = 0; i < (int)target_set.size(); i++) {
        if (i == ego_idx) continue;
        if (isFiniteVec2(target_set[i])) {
            if (std::hypot(x - target_set[i].first, y - target_set[i].second) < inter_robot_threshold_) return true;
        }
    }
    return false;
}

bool FrontierAssigner::CheckIRCollision(const int& ego_idx, const double& x, const double& y) {
    // std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < (int)poses_.size(); i++) {
        if (i == ego_idx) continue;
        if (std::hypot(poses_[i].position.x - x, poses_[i].position.y - y) < inter_robot_threshold_) return true;
    }
    return false;
}

bool FrontierAssigner::CheckFrontierFeasibility(const std::vector<Vec2>& frontiers, const Vec2& candidate) {
    if (frontiers.empty()) return true;
    for (const auto& f : frontiers) {
        if (dist(f, candidate) < 4.0) return true;
    }
    return false;
}

bool FrontierAssigner::ReachedTarget(int r, const Vec2& target, double radius) const {
    if (!std::isfinite(target.first) || !std::isfinite(target.second)) return false;
    // std::lock_guard<std::mutex> lock(mtx_);
    const double dx = poses_[r].position.x - target.first;
    const double dy = poses_[r].position.y - target.second;
    ROS_INFO("Distance to the Goal is %f", std::hypot(dx, dy));
    return std::hypot(dx, dy) <= radius;
}

bool FrontierAssigner::FrontierDirectionCheck(int r_idx,
                                              const Vec2& candidate,
                                              int g_idx,
                                              const Vec2& g_target) const
{
    // std::lock_guard<std::mutex> lock(mtx_);
    if (r_idx < 0 || r_idx >= num_robot_ || g_idx < 0 || g_idx >= num_robot_) return false;

    const double vrx = candidate.first  - poses_[r_idx].position.x;
    const double vry = candidate.second - poses_[r_idx].position.y;

    const double vgx = g_target.first  - poses_[g_idx].position.x;
    const double vgy = g_target.second - poses_[g_idx].position.y;

    const double nr = std::hypot(vrx, vry);
    const double ng = std::hypot(vgx, vgy);

    if (nr < 1e-6 || ng < 1e-6) return true;

    double c = (vrx * vgx + vry * vgy) / (nr * ng);
    c = std::max(-1.0, std::min(1.0, c));

    const double theta = std::acos(c);
    const double theta_max = max_heading_angle_deg_ * M_PI / 180.0;
    return theta <= theta_max;
}

// keep a simple, stable cost for ORANGE cell selection
double FrontierAssigner::computeCellCost(int& ego_idx, int& ri_idx, const Vec2& candidate_cell, const bool& emergency) const {
    // std::lock_guard<std::mutex> lock(mtx_);
    if (ego_idx < 0 || ego_idx >= num_robot_ || ri_idx < 0 || ri_idx >= num_robot_) return 1e9;

    // const auto yaw_ref  = yawFromTo(poses_[ego_idx], {poses_[ri_idx].position.x, poses_[ri_idx].position.y});
    // const auto yaw_this = yawFromTo(poses_[ego_idx], candidate_cell);

    // double d = std::atan2(std::sin(yaw_ref - yaw_this), std::cos(yaw_ref - yaw_this));
    // double delta_yaw = std::fabs(d);
    // if (delta_yaw > M_PI / 2) return 1e5;

    // if (emergency) {
    //     auto los_info = scp_->starConvexConstraint({poses_[ri_idx].position.x, poses_[ri_idx].position.y}, {poses_[ego_idx].position.x, poses_[ego_idx].position.y}, 
    //                                robots_convexhull_[ri_idx], false);
    //     return los_info[0];
    // }
    // else {
        // prefer closer
        return std::hypot(candidate_cell.first - poses_[ego_idx].position.x,
                        candidate_cell.second - poses_[ego_idx].position.y);
    // }
}

double FrontierAssigner::computeLoSCost(int& ego_idx, int& ri_idx) const {
    (void)ego_idx; (void)ri_idx;
    return 0.0;
}

bool FrontierAssigner::EvaluateNeiboringTarget(int& ego_idx, int& r_idx, Vec2& tmp,
                                               const std::vector<Vec2>& target_set, const bool& emergency) {
    // std::lock_guard<std::mutex> lock(mtx_);

    if (r_idx < 0 || r_idx >= num_robot_) return false;

    if (std::hypot(poses_[ego_idx].position.x - poses_[r_idx].position.x,
                   poses_[ego_idx].position.y - poses_[r_idx].position.y) <= 1.5) {
        tmp = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};
        return true;
    }

    const auto& pc = robots_neighbor_pc_[r_idx];
    if (pc.empty()) {
        tmp = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};
        return false;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    Vec2 best_pt = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};

    for (const auto& r : pc) {
        // if (CheckExistingTargets(ego_idx, r.x(), r.y(), target_set)) continue;
        // if (CheckIRCollision(ego_idx, r.x(), r.y())) continue;

        Vec2 cand{r.x(), r.y()};
        double c = computeCellCost(ego_idx, r_idx, cand, emergency);
        if (!std::isfinite(c)) continue;
        if (c < best_cost) {
            best_cost = c;
            best_pt = cand;
        }
    }

    if (!std::isfinite(best_cost)) {
        tmp = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};
        return false;
    }
    tmp = best_pt;
    return true;
}

int FrontierAssigner::selectNeighborRefRobot(
    int r,
    const std::vector<int>& nei_idx_set,
    const std::vector<Vec2>& assigned_targets_xy,
    const std::vector<bool>& is_temp) const
{
    const Vec2 r_pos = poseXY(poses_[r]);

    // 1) among neighbors with GREEN targets, choose the FURTHEST such target
    int best_idx = -1;
    double best_d = std::numeric_limits<double>::infinity();

    for (int n : nei_idx_set) {
        if (n < 0 || n >= num_robot_ || n == r) continue;
        if (!isFiniteVec2(assigned_targets_xy[n]) || is_temp[n]) continue; // n is GREEN
        const double d = dist(r_pos, assigned_targets_xy[n]);
        if (d < best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    if (best_idx != -1) return best_idx;

    // 2) otherwise choose a neighbor robot:
    //    prefer neighbors whose neighbors have GREEN targets;
    //    tie-break by (#second-hop green), then by distance (FURTHEST)
    auto secondHopGreenCount = [&](int n)->int {
        if (n < 0 || n >= num_robot_) return 0;
        if ((int)robot_neighbors_.size() != num_robot_) return 0;

        int cnt = 0;
        const auto& n_neis = robot_neighbors_[n];
        for (int m : n_neis) {
            if (m < 0 || m >= num_robot_) continue;
            if (m == r) continue; // ignore going back to self
            if (m >= (int)is_temp.size()) continue;
            if (!isFiniteVec2(assigned_targets_xy[m])) continue;
            if (is_temp[m]) continue; // m is not GREEN
            cnt++;
        }
        return cnt;
    };

    // Step2-A: among direct neighbors, first maximize second-hop-green-count, then distance
    best_idx = -1;
    int best_cnt = -1;
    best_d = -std::numeric_limits<double>::infinity();

    for (int n : nei_idx_set) {
        if (n < 0 || n >= num_robot_ || n == r) continue;

        const int cnt = secondHopGreenCount(n);
        const Vec2 n_pos = poseXY(poses_[n]);
        const double d = dist(r_pos, n_pos);

        // Only apply this "2-hop green" preference when cnt>0
        if (cnt <= 0) continue;

        if (cnt > best_cnt || (cnt == best_cnt && d > best_d)) {
            best_cnt = cnt;
            best_d   = d;
            best_idx = n;
        }
    }
    if (best_idx != -1) return best_idx;

    // Step2-B: if nobody has 2-hop GREEN, fallback to original "FURTHEST neighbor robot"
    best_idx = -1;
    best_d = -std::numeric_limits<double>::infinity();
    for (int n : nei_idx_set) {
        if (n < 0 || n >= num_robot_ || n == r) continue;
        const Vec2 n_pos = poseXY(poses_[n]);
        const double d = dist(r_pos, n_pos);
        if (d > best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    if (best_idx != -1) return best_idx;

    // 3) fallback global FURTHEST (excluding self)
    best_d = -std::numeric_limits<double>::infinity();
    for (int n = 0; n < num_robot_; ++n) {
        if (n == r) continue;
        const Vec2 n_pos = poseXY(poses_[n]);
        const double d = dist(r_pos, n_pos);
        if (d > best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    return best_idx;
}

// ============================================================
// NEW assignCloseFrontiers (your new pipeline)
// ============================================================

FrontierAssigner::AssignResult FrontierAssigner::assignCloseFrontiers() {
    std_msgs::Float64MultiArray record_traj_msg;
    record_traj_msg.data.resize(num_robot_ * 5 + 1);
    AssignResult out;
    out.leader_idx = -1;
    out.targets_xy.assign(num_robot_, NaN2());
    out.navigation_done = false;

    const Snapshot snap = getSnapshotLocked();
    if ((int)snap.poses.size() != num_robot_) return out;
    auto unitDir = [&](const Vec2& from, const Vec2& to)->std::pair<double,double> {
        double vx = to.first - from.first;
        double vy = to.second - from.second;
        double n  = std::hypot(vx, vy);
        if (!std::isfinite(n) || n < 1e-6) return {0.0, 0.0};
        return {vx / n, vy / n};
    };

    auto angleBetween = [&](const std::pair<double,double>& a,
                            const std::pair<double,double>& b)->double {
        double c = a.first*b.first + a.second*b.second;
        if (!std::isfinite(c)) return 0.0;
        c = std::max(-1.0, std::min(1.0, c));
        return std::acos(c);
    };

    // -----------------------------------------
    // Step 0: emergency stop -> permanently block last published/last remembered target, clear memory
    // -----------------------------------------
    // {
    //     // std::lock_guard<std::mutex> lock(mtx_);
    //     for (int r = 0; r < num_robot_; ++r) {
    //         if ((int)snap.emerg_stop.size() != num_robot_) break;
    //         if (!snap.emerg_stop[r]) continue;

    //         Vec2 bad = NaN2();
    //         if (has_published_goal_[r] && isFiniteVec2(last_published_goal_[r])) bad = last_published_goal_[r];
    //         else if (has_target_[r] && isFiniteVec2(last_targets_xy_[r])) bad = last_targets_xy_[r];
    //         else if (r < (int)snap.last_targets.size() && isFiniteVec2(snap.last_targets[r])) bad = snap.last_targets[r];

    //         if (isFiniteVec2(bad)) {
    //             bool existed = false;
    //             for (const auto& bf : perm_block_frontiers_) {
    //                 if (dist(bad, bf) < perm_block_merge_eps_) { existed = true; break; }
    //             }
    //             if (!existed) perm_block_frontiers_.push_back(bad);

    //             ROS_WARN("\033[41;37m EmergencyStop r%d: permanently block target (%.2f, %.2f)\033[0m",
    //                     r + 1, bad.first, bad.second);
    //         }

    //         // clear memory to avoid resurrecting
    //         has_target_[r] = false;
    //         last_targets_xy_[r] = NaN2();
    //         last_is_temp_[r] = true;

    //         has_green_target_[r] = false;
    //         last_green_targets_xy_[r] = NaN2();
    //     }
    // }

    // -----------------------------------------
    // Step A: build merged+cluster frontiers, filter perm blocked
    // -----------------------------------------
    auto isPermBlocked = [&](const Vec2& f)->bool {
        for (const auto& bf : perm_block_frontiers_) {
            if (dist(f, bf) < perm_block_merge_eps_) return true;
        }
        return false;
    };

    std::vector<Vec2> raw;
    raw.reserve(2048);
    for (int i = 0; i < num_robot_; ++i) {
        for (const auto& f : snap.frontiers_by_robot[i]) raw.push_back(f);
    }
    // Add retry-frontiers into the candidate frontier pool
    for (const auto& f : stuck_retry_frontiers_) {
        if (isFiniteVec2(f)) raw.push_back(f);
    }
    // -----------------------------------------
    // NEW: filter out frontiers that are too close to ANY robot (< 1.5m)
    // -----------------------------------------
    const double frontier_robot_min_dist = frontier_robot_min_dist_;
    const double frontier_robot_min_dist2 = frontier_robot_min_dist * frontier_robot_min_dist;
    auto behindCloseAnyRobot = [&](const Vec2& f)->bool {
        if (!isFiniteVec2(f)) return true;

        for (int r = 0; r < num_robot_; ++r) {
            const Vec2 pr = poseXY(snap.poses[r]);
            const double dx = f.first  - pr.first;
            const double dy = f.second - pr.second;
            const double d  = std::hypot(dx, dy);
            if (!std::isfinite(d) || d < 1e-6) continue;

            if (d >= 3.0) continue; // only care when close

            const double yaw = tf::getYaw(snap.poses[r].orientation);
            const double fx = std::cos(yaw);
            const double fy = std::sin(yaw);

            const double cosang = (dx*fx + dy*fy) / d;  // normalized
            if (cosang < 0.0) return true;              // behind (>90deg)
        }
        return false;
    };
    auto tooCloseToAnyRobot = [&](const Vec2& f)->bool {
        for (int r = 0; r < num_robot_; ++r) {
            const double dx = f.first  - snap.poses[r].position.x;
            const double dy = f.second - snap.poses[r].position.y;
            const double d2 = dx*dx + dy*dy;
            if (d2 < frontier_robot_min_dist2) return true;
        }
        return false;
    };

    // Apply filter
    {
        std::vector<Vec2> filtered;
        filtered.reserve(raw.size());
        for (const auto& f : raw) {
            if (!isFiniteVec2(f)) continue;

            // NEW: drop frontiers near reached-GREEN history
            if (nearAnyAnchorHistory(f)) continue;

            // existing: hard reject if behind+close to ANY robot
            if (behindCloseAnyRobot(f)) continue;

            // existing: reject if too close to any robot
            if (tooCloseToAnyRobot(f)) continue;

            filtered.push_back(f);
        }
        raw.swap(filtered);
    }
    // filter perm blocked
    if (!perm_block_frontiers_.empty()) {
        std::vector<Vec2> filtered;
        filtered.reserve(raw.size());
        for (const auto& f : raw) if (!isPermBlocked(f)) filtered.push_back(f);
        raw.swap(filtered);
    }
// -----------------------------------------
// NEW: near-center (<=10m) clustering with radius=3m (connected components),
//      choose ONE representative per cluster.
//      far (>10m): keep fine de-dup with frontier_merge_eps_
// -----------------------------------------

// 1) team center
Vec2 team_center{0.0, 0.0};
for (int r = 0; r < num_robot_; ++r) {
    team_center.first  += snap.poses[r].position.x;
    team_center.second += snap.poses[r].position.y;
}
team_center.first  /= (double)num_robot_;
team_center.second /= (double)num_robot_;

// 2) params
const double near_range = cluster_window_;              // within 10m from team center
const double cluster_r  = cluster_radius_;               // clustering radius
const double eps_far    = frontier_merge_eps_; // e.g. 0.35

const double near_range2 = near_range * near_range;
const double cluster_r2  = cluster_r  * cluster_r;
const double eps_far2    = eps_far    * eps_far;

auto dist2 = [&](const Vec2& a, const Vec2& b)->double {
    const double dx = a.first - b.first;
    const double dy = a.second - b.second;
    return dx*dx + dy*dy;
};
auto isNear = [&](const Vec2& p)->bool {
    return dist2(p, team_center) <= near_range2;
};

// 3) split raw into near/far
std::vector<Vec2> near_pts;
std::vector<Vec2> far_pts;
near_pts.reserve(raw.size());
far_pts.reserve(raw.size());

for (const auto& p : raw) {
    if (!isFiniteVec2(p)) continue;
    if (isNear(p)) near_pts.push_back(p);
    else           far_pts.push_back(p);
}

// 4) far: fine de-dup (same as your old logic but using eps_far)
std::vector<Vec2> far_frontiers;
far_frontiers.reserve(far_pts.size());
for (const auto& p : far_pts) {
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < (int)far_frontiers.size(); ++i) {
        const double d2 = dist2(p, far_frontiers[i]);
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    if (best != -1 && best_d2 < eps_far2) continue;
    far_frontiers.push_back(p);
}

// 5) near: radius-3m connected clustering (BFS/Union-Find style)
//    Each cluster outputs ONE representative point.
//    Representative strategy here: pick the point in the cluster that is closest to team_center.
//    (更稳、更符合“就近推进探索”)
std::vector<Vec2> near_frontiers;
near_frontiers.reserve(near_pts.size());

const int N = (int)near_pts.size();
std::vector<char> visited(N, 0);

for (int i = 0; i < N; ++i) {
    if (visited[i]) continue;
    visited[i] = 1;

    // BFS queue for this cluster
    std::vector<int> q;
    q.reserve(64);
    q.push_back(i);

    // cluster members
    std::vector<int> members;
    members.reserve(64);
    members.push_back(i);

    // BFS expansion: connect if distance <= 3m
    for (size_t qi = 0; qi < q.size(); ++qi) {
        const int u = q[qi];
        const Vec2& pu = near_pts[u];

        for (int v = 0; v < N; ++v) {
            if (visited[v]) continue;
            if (dist2(pu, near_pts[v]) <= cluster_r2) {
                visited[v] = 1;
                q.push_back(v);
                members.push_back(v);
            }
        }
    }

    // pick representative: closest to team_center
    int best_idx = members[0];
    double best_d2 = dist2(near_pts[best_idx], team_center);
    for (int idx : members) {
        const double d2c = dist2(near_pts[idx], team_center);
        if (d2c < best_d2) {
            best_d2 = d2c;
            best_idx = idx;
        }
    }
    near_frontiers.push_back(near_pts[best_idx]);
}

// 6) merge near representatives + far frontiers
std::vector<Vec2> frontiers;
frontiers.reserve(near_frontiers.size() + far_frontiers.size());
frontiers.insert(frontiers.end(), near_frontiers.begin(), near_frontiers.end());
frontiers.insert(frontiers.end(), far_frontiers.begin(), far_frontiers.end());

// (可选) 最后再做一次全局细去重，避免 near/far 边界附近重复
{
    std::vector<Vec2> final_frontiers;
    final_frontiers.reserve(frontiers.size());
    for (const auto& p : frontiers) {
        int best = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (int i = 0; i < (int)final_frontiers.size(); ++i) {
            const double d2 = dist2(p, final_frontiers[i]);
            if (d2 < best_d2) { best_d2 = d2; best = i; }
        }
        if (best != -1 && best_d2 < eps_far2) continue; // 用细 eps 做最终去重
        final_frontiers.push_back(p);
    }
    frontiers.swap(final_frontiers);
}

// frontiers.push_back({7, 3});
// -----------------------------------------
// Visualize clustered frontiers (purple spheres)
// -----------------------------------------
{
    visualization_msgs::Marker mk =
        makeSphereListMarker("clustered_frontiers", 0, makeColor(0.6f, 0.0f, 0.8f, 1.0f)); // purple
    mk.scale.x = 0.25;  // 小一点更像候选点
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;

    mk.points.reserve(frontiers.size());
    for (const auto& f : frontiers) {
        if (!isFiniteVec2(f)) continue;
        geometry_msgs::Point p;
        p.x = f.first;
        p.y = f.second;
        p.z = 0.05;
        mk.points.push_back(p);
    }

    pub_clustered_frontiers_marker_.publish(mk);
}
    // -----------------------------------------
    // Anchor LoS-lock maintenance (before selecting anchor)
    // -----------------------------------------
    // auto findMatchedFrontier = [&](const Vec2& ref, Vec2& matched)->bool {
    //     if (!isFiniteVec2(ref)) return false;
    //     double best = std::numeric_limits<double>::infinity();
    //     int best_k = -1;
    //     for (int k = 0; k < (int)frontiers.size(); ++k) {
    //         const double d = dist(frontiers[k], ref);
    //         if (d < best) { best = d; best_k = k; }
    //     }
    //     if (best_k >= 0 && best <= anchor_lock_match_eps_) {
    //         matched = frontiers[best_k];
    //         return true;
    //     }
    //     return false;
    // };

    // bool anchor_lock_valid_this_round = false;
    // Vec2 anchor_lock_target_now = NaN2();

    // if (anchor_lock_active_ &&
    //     anchor_lock_robot_ >= 0 && anchor_lock_robot_ < num_robot_ &&
    //     isFiniteVec2(anchor_lock_target_))
    // {
    //     // exit if reached
    //     if (ReachedTarget(anchor_lock_robot_, anchor_lock_target_, reach_radius_)) {
    //         anchor_lock_active_ = false;
    //         anchor_lock_robot_  = -1;
    //         anchor_lock_target_ = NaN2();
    //     } else if (findMatchedFrontier(anchor_lock_target_, anchor_lock_target_now)) {
    //         anchor_lock_valid_this_round = true;
    //     } else {
    //         // target disappeared / changed too much
    //         anchor_lock_active_ = false;
    //         anchor_lock_robot_  = -1;
    //         anchor_lock_target_ = NaN2();
    //     }
    // }
    // -----------------------------------------
    // Anchor LoS-lock DISABLED
    // -----------------------------------------
    bool anchor_lock_valid_this_round = false;
    Vec2 anchor_lock_target_now = NaN2();
    anchor_lock_active_ = false;
    anchor_lock_robot_  = -1;
    anchor_lock_target_ = NaN2();
    // -----------------------------------------
    // If no frontiers: no freeze -> all ORANGE (neighbor PC or stay)
    // -----------------------------------------
    if (frontiers.empty()) {
        std::vector<bool> is_temp(num_robot_, true);
        for (int r = 0; r < num_robot_; ++r) {
            Vec2 tmp = poseXY(snap.poses[r]);
            std::vector<int> nei = (snap.neighbors.size() == (size_t)num_robot_) ? snap.neighbors[r] : std::vector<int>{};
            while (!nei.empty()) {
                int ref = selectNeighborRefRobot(r, nei, out.targets_xy, is_temp);
                nei.erase(std::remove(nei.begin(), nei.end(), ref), nei.end());
                if (ref < 0 || ref >= num_robot_ || ref == r) continue;
                int rr = ref;
                int ego = r;
                if (EvaluateNeiboringTarget(ego, rr, tmp, out.targets_xy, snap.emerg_stop[ego])) break;
            }
            out.targets_xy[r] = tmp;
        }

        // publish
        for (int r = 0; r < num_robot_; ++r) publishMoveBaseSimpleGoal(r, out.targets_xy[r], snap.poses[r]);

        // marker
        visualization_msgs::Marker mk_temp = makeSphereListMarker("temp_targets", 0, makeColor(1.0f, 0.55f, 0.0f, 1.0f));
        for (int r = 0; r < num_robot_; ++r) {
            geometry_msgs::Point p; p.x = out.targets_xy[r].first; p.y = out.targets_xy[r].second; p.z = 0.05;
            mk_temp.points.push_back(p);
        }
        pub_temp_targets_marker_.publish(mk_temp);

        // memory update
        {
            // std::lock_guard<std::mutex> lock(mtx_);
            for (int r = 0; r < num_robot_; ++r) {
                last_targets_xy_[r] = out.targets_xy[r];
                last_is_temp_[r] = true;
                has_target_[r] = true;
                last_published_goal_[r] = out.targets_xy[r];
                has_published_goal_[r] = true;
                last_published_is_temp_[r] = true;
            }
        }
        return out;
    }

    auto losPenalty = [&](const Vec2& a, const Vec2& b)->double {
        // LoS clear => 0; blocked => los_penalty_
        if (!isFiniteVec2(a) || !isFiniteVec2(b)) return los_penalty_;
        return isLineFreeOnMap(a, b) ? 0.0 : los_penalty_;
    };

    auto robotToFrontierCost = [&](int r, const Vec2& f)->double {
        const Vec2 pr = poseXY(snap.poses[r]);
        // 这里可以继续叠加你其他 hard filter，比如 forbid/behind-close 之类
        return dist(pr, f) 
        + losPenalty(pr, f);
    };

    // // -----------------------------------------
    // // Team center
    // // -----------------------------------------
    // Vec2 center{0.0, 0.0};
    // for (int r = 0; r < num_robot_; ++r) {
    //     center.first  += snap.poses[r].position.x;
    //     center.second += snap.poses[r].position.y;
    // }
    // center.first  /= (double)num_robot_;
    // center.second /= (double)num_robot_;

    // -----------------------------------------
    // Step G1: anchor = globally closest (robot, frontier) pair
    // -----------------------------------------
    int r_anchor = -1;
    int k_anchor = -1;
    Vec2 f_anchor = NaN2();

    // if (anchor_lock_valid_this_round) {
    //     r_anchor = anchor_lock_robot_;
    //     f_anchor = anchor_lock_target_now;
    // } else {
        double best = std::numeric_limits<double>::infinity();
        for (int r = 0; r < num_robot_; ++r) {
            const Vec2 pr = poseXY(snap.poses[r]);
            for (int k = 0; k < (int)frontiers.size(); ++k) {
                if (isPermBlocked(frontiers[k])) continue;
                if (isRobotForbidden(r, frontiers[k])) continue;

                // 如果你已经在 merge 阶段做了 behind-close 过滤，这里不需要再写 isBehindAndClose
                // 如果你还保留着也没关系：if (isBehindAndClose(r, frontiers[k])) continue;

                const double c = robotToFrontierCost(r, frontiers[k]);
                // const double c = dist2(pr, frontiers[k]);
                if (c < best) {
                    best = c;
                    r_anchor = r;
                    k_anchor = k;
                    f_anchor = frontiers[k];
                }
            }
        }
    // }

    // safety fallback (shouldn't happen, but prevent crash)
    // if (r_anchor < 0 || !isFiniteVec2(f_anchor)) {
    //     // fallback to "no frontiers" behavior
    //     return /* 你现有的 All-ORANGE by neighbor LOS 分支 */;
    // }

    // Candidate GREEN assignment
    std::vector<Vec2> green_targets(num_robot_, NaN2());
    std::vector<bool> is_green(num_robot_, false);

    // Fix the anchor assignment
    green_targets[r_anchor] = f_anchor;
    // -----------------------------------------
    // Anchor LoS-lock trigger: if anchor->target is blocked, lock this assignment
    // -----------------------------------------
    // {
    //     const Vec2 p_anchor = poseXY(snap.poses[r_anchor]);
    //     if (isFiniteVec2(p_anchor) && isFiniteVec2(f_anchor)) {
    //         const bool los_ok = isLineFreeOnMap(p_anchor, f_anchor);
    //         if (!los_ok) {
    //             anchor_lock_active_ = true;
    //             anchor_lock_robot_  = r_anchor;
    //             anchor_lock_target_ = f_anchor;
    //         }
    //     }
    // }
    is_green[r_anchor] = true;

    // ===== Retry-mode injection: lock retry target as a mandatory GREEN =====
    int retry_guard_robot = -1;
    Vec2 retry_guard_target = NaN2();
    bool retry_guard_valid = false;

    if (retry_active_ && retry_robot_ >= 0 && retry_robot_ < num_robot_ && isFiniteVec2(retry_target_)) {
        // 若已到达 -> 关闭 retry
        const Vec2 pr = poseXY(snap.poses[retry_robot_]);
        if (dist(pr, retry_target_) <= reach_radius_) {
            ROS_WARN("\033[32m Retry target reached by r%d -> exit retry mode \033[0m", retry_robot_ + 1);
            retry_active_ = false;
            retry_robot_ = -1;
            retry_target_ = NaN2();
        } else {
            retry_guard_robot  = retry_robot_;
            retry_guard_target = retry_target_;
            retry_guard_valid  = true;

            // 强制把 retry_robot_ 设成 GREEN，并锁定目标
            green_targets[retry_guard_robot] = retry_guard_target;
            is_green[retry_guard_robot]      = true;

            // 关键：避免 anchor 与 retry_robot 冲突（同一机器人）
            if (retry_guard_robot == r_anchor) {
                // anchor 机器人现在以 retry 目标为准
                k_anchor = -1;              // 仅标记，无需再用
                // f_anchor 变量你后面还会用的话，最好同步
                // f_anchor = retry_guard_target; // 若 f_anchor 是 const 就别动
            } else {
                // anchor 机器人仍然保持 anchor 目标，但后面 Hungarian 要排除 retry 目标（见下面）
            }

            ROS_WARN("\033[38;5;208m Retry mode ON: lock r%d -> (%.2f, %.2f)\033[0m",
                    retry_guard_robot + 1, retry_guard_target.first, retry_guard_target.second);
        }
    }

    // -----------------------------------------
    // Step JUMP-GUARD: anchor jump detection + pause K cycles
    // -----------------------------------------
    bool paused_this_round = false; 
    {
        bool anchor_jump = false;

        if (has_last_round_green_ref_) {
            const double d = dist(f_anchor, last_round_green_ref_);
            auto cur_dir = unitDir({snap.poses[r_anchor].position.x, snap.poses[r_anchor].position.y}, f_anchor);

            double theta_deg = 0.0;
            if (has_last_round_green_dir_) {
                theta_deg = angleBetween(last_round_green_dir_, cur_dir) * 180.0 / M_PI;
            }

            if (d > jump_guard_dist_th_ 
                // || theta_deg > jump_guard_yaw_th_
            ) {
                anchor_jump = true;
                ROS_WARN("\033[38;5;208m Anchor jump suspected: d=%.2f(th=%.2f), theta=%.1fdeg(th=%.1f) \033[0m",
                        d, jump_guard_dist_th_, theta_deg, jump_guard_yaw_th_);
            }

            // If currently pausing, allow early exit when anchor becomes normal
            if (anchor_pause_ct_ > 0 && !anchor_jump) {
                ROS_WARN("\033[32m Anchor recovered during pause -> resume exploration \033[0m");
                anchor_pause_ct_ = 0;
            }
        }

        // Trigger / continue pause
        if (anchor_pause_ct_ > 0 || (anchor_jump && !allow_anchor_jump_)) {
            if (anchor_pause_ct_ == 0 && anchor_jump) {
                anchor_pause_ct_ = std::max(1, anchor_pause_K_);
                ROS_WARN("\033[38;5;208m Enter anchor-wait pause for K=%d cycles \033[0m", anchor_pause_ct_);
            }

        if (anchor_pause_ct_ > 0) {
            anchor_pause_ct_--;
            if (anchor_pause_ct_ == 0) {
                allow_anchor_jump_ = true;
                ROS_WARN("\033[38;5;208m Pause expired -> allow anchor jump and explore other region \033[0m");
            }

            paused_this_round = true;
            ROS_WARN("\033[38;5;208m Pause... remaining=%d (publish stay-goals) \033[0m", anchor_pause_ct_);

            // publish stay goals as ORANGE
            std::vector<bool> is_temp(num_robot_, true);
            for (int r = 0; r < num_robot_; ++r) {
                out.targets_xy[r] = poseXY(snap.poses[r]); // stay
                publishMoveBaseSimpleGoal(r, out.targets_xy[r], snap.poses[r]);
            }

            // markers
            visualization_msgs::Marker mk_temp =
                makeSphereListMarker("temp_targets", 0, makeColor(1.0f, 0.55f, 0.0f, 1.0f));
            for (int r = 0; r < num_robot_; ++r) {
                geometry_msgs::Point p;
                p.x = out.targets_xy[r].first;
                p.y = out.targets_xy[r].second;
                p.z = 0.05;
                mk_temp.points.push_back(p);
            }
            pub_temp_targets_marker_.publish(mk_temp);

            // target type: all false (ORANGE)
            std_msgs::Bool msg; msg.data = false;
            for (int r = 0; r < num_robot_; ++r) pub_targets_type_[r].publish(msg);

            // NOTE: do NOT update last_round_green_ref_/dir here (we are waiting)
            return out;
        }
        }
    }

    // -----------------------------------------
    // Step G2: Hungarian for remaining robots and remaining frontiers (min total distance)
    // -----------------------------------------
    auto isSameAsAnchorTarget = [&](const Vec2& f)->bool {
        return isFiniteVec2(f_anchor) && dist(f, f_anchor) < goal_dedup_eps_;
    };
    std::vector<int> robots_rest;
    robots_rest.reserve(num_robot_ - 1);
    for (int r = 0; r < num_robot_; ++r) {
        if (r == r_anchor) continue;
        if (retry_guard_valid && r == retry_guard_robot) continue;
        robots_rest.push_back(r);
    }

    std::vector<int> frontier_rest;
    frontier_rest.reserve((int)frontiers.size() - 1);
    for (int k = 0; k < (int)frontiers.size(); ++k) {
        // if (k == k_anchor) continue;
        if (isSameAsAnchorTarget(frontiers[k])) continue;
        if (retry_guard_valid && dist(frontiers[k], retry_guard_target) < goal_dedup_eps_) continue;
        frontier_rest.push_back(k);
    }

    const int nR = (int)robots_rest.size();
    const int nF = (int)frontier_rest.size();
    const int n  = std::max(nR, nF);
    const double BIG = 1e6;

    std::vector<std::vector<double>> cost(n, std::vector<double>(n, BIG));
    for (int i = 0; i < nR; ++i) {
        int r = robots_rest[i];
        Vec2 pr = poseXY(snap.poses[r]);
        for (int j = 0; j < nF; ++j) {
            Vec2 fk = frontiers[ frontier_rest[j] ];
            const double c = dist(pr, fk)
            + losPenalty(pr, fk);
            cost[i][j] = c;
        }
    }

    std::vector<int> row_to_col = HungarianMinCost::Solve(cost);
    for (int i = 0; i < nR; ++i) {
        int r = robots_rest[i];
        int j = (i < (int)row_to_col.size()) ? row_to_col[i] : -1;
        if (j >= 0 && j < nF) {
            int k = frontier_rest[j];
            if (isRobotForbidden(r, frontiers[k])) continue; // let it stay non-green this round
            green_targets[r] = frontiers[k];
            is_green[r] = true;
        }
    }

    // -----------------------------------------
    // Constraint checking helpers (GREEN filtering)
    // -----------------------------------------
    auto directionOK = [&](int r_idx, const Vec2& r_tgt, int g_idx, const Vec2& g_tgt)->bool {
        if (!isFiniteVec2(r_tgt) || !isFiniteVec2(g_tgt)) return false;
        auto vr = dirUnitOrYaw(snap.poses[r_idx], r_tgt);
        auto vg = dirUnitOrYaw(snap.poses[g_idx], g_tgt);
        double c = vr.first*vg.first + vr.second*vg.second;
        if (!std::isfinite(c)) return false;
        c = clamp01(c);
        double theta = std::acos(c);
        double th = max_heading_angle_deg_ * M_PI / 180.0;
        return theta <= th;
    };

    auto checkIRCollisionSnap = [&](int ego, const Vec2& p)->bool {
        for (int i = 0; i < num_robot_; ++i) {
            if (i == ego) continue;
            if (std::hypot(snap.poses[i].position.x - p.first,
                           snap.poses[i].position.y - p.second) < 1.8) return true;
        }
        return false;
    };

    auto goalDedupOK = [&](const Vec2& cand, int self_r)->bool {
        for (int g = 0; g < num_robot_; ++g) {
            if (g == self_r) continue;
            if (!is_green[g]) continue;
            if (!isFiniteVec2(green_targets[g])) continue;
            if (dist(cand, green_targets[g]) < goal_dedup_eps_) return false;
        }
        return true;
    };

    auto pairwiseOKWithCurrentGreens = [&](int r, const Vec2& cand)->bool {
        for (int g = 0; g < num_robot_; ++g) {
            if (g == r) continue;
            if (!is_green[g]) continue;
            if (!isFiniteVec2(green_targets[g])) continue;

            // direction both ways
            if (!directionOK(r, cand, g, green_targets[g])) return false;
            if (!directionOK(g, green_targets[g], r, cand)) return false;

            // distance constraints
            const double d = dist(cand, green_targets[g]);
            if (d < MIN_TGT_DIST || d > MAX_TGT_DIST) {
                ROS_INFO("Distance Constraints Violated!");
                return false;
            }
            // LoS
            if (!isLineFreeOnMap(cand, green_targets[g])) return false;

            // extra dedup
            if (d < goal_dedup_eps_) return false;
        }
        return true;
    };

    // auto unitDirFromPoseToTarget = [&](int r, const Vec2& tgt)->std::pair<double,double> {
    //     const Vec2 p = poseXY(snap.poses[r]);
    //     double vx = tgt.first - p.first;
    //     double vy = tgt.second - p.second;
    //     double n = std::hypot(vx, vy);
    //     if (!std::isfinite(n) || n < 1e-6) {
    //         // fallback pose yaw
    //         double yaw = tf::getYaw(snap.poses[r].orientation);
    //         return {std::cos(yaw), std::sin(yaw)};
    //     }
    //     return {vx/n, vy/n};
    // };

    // auto angleDegBetween = [&](const std::pair<double,double>& a,
    //                         const std::pair<double,double>& b)->double {
    //     double c = a.first*b.first + a.second*b.second;
    //     if (!std::isfinite(c)) return 0.0;
    //     c = std::max(-1.0, std::min(1.0, c));
    //     return std::acos(c) * 180.0 / M_PI;
    // };

    const Vec2 t_anchor = green_targets[r_anchor];
    const bool anchor_valid = isFiniteVec2(t_anchor) &&
                            !isPermBlocked(t_anchor) &&
                            !isRobotForbidden(r_anchor, t_anchor);

    auto unitDirFromPoseToTarget = [&](int rr, const Vec2& tgt)->std::pair<double,double> {
        const Vec2 p = poseXY(snap.poses[rr]);
        double vx = tgt.first - p.first, vy = tgt.second - p.second;
        double n = std::hypot(vx, vy);
        if (!std::isfinite(n) || n < 1e-6) {
            double yaw = tf::getYaw(snap.poses[rr].orientation);
            return {std::cos(yaw), std::sin(yaw)};
        }
        return {vx/n, vy/n};
    };

    auto angleDegBetween = [&](const std::pair<double,double>& a,
                            const std::pair<double,double>& b)->double {
        double c = a.first*b.first + a.second*b.second;
        c = std::max(-1.0, std::min(1.0, c));
        return std::acos(c) * 180.0 / M_PI;
    };

    std::pair<double,double> anchor_dir{0.0, 0.0};
    if (anchor_valid) anchor_dir = unitDirFromPoseToTarget(r_anchor, t_anchor);

    // 可调阈值（建议做成 rosparam）
    // const double anchor_gate_dist = 20.0;   // 目标离 anchor 目标太远则拒绝
    // const double anchor_gate_ang  = 70.0;  // 目标方向相对 anchor 方向偏离太大则拒绝

    auto anchorRobotDistOK = [&](int r)->bool {
        if (r_anchor < 0 || r_anchor >= num_robot_) return true;   // no anchor => don't block
        if (r == r_anchor) return true;                            // anchor itself always ok

        const Vec2 pa = poseXY(snap.poses[r_anchor]);
        const Vec2 pr = poseXY(snap.poses[r]);

        if (!isFiniteVec2(pa) || !isFiniteVec2(pr)) return true;   // don't block if bad pose

        return dist(pa, pr) <= max_green_robot_anchor_dist_;
    };

    auto canKeepGreen = [&](int r, const Vec2& cand)->bool {
        if (!isFiniteVec2(cand)) return false;
        if (isPermBlocked(cand)) return false;
        if (isRobotForbidden(r, cand)) return false;
        if (snap.emerg_stop[r] == true) return false;
        // NEW: anchor-relative hard gate (optional but recommended)
        // if (anchor_valid && r != r_anchor) {
            // distance to anchor target
            // if (dist(cand, t_anchor) > anchor_gate_dist) return false;

            // direction consistency w.r.t anchor direction
            // const auto rdir = unitDirFromPoseToTarget(r, cand);
            // const double ang = angleDegBetween(anchor_dir, rdir);
            // if (ang > anchor_gate_ang) return false;
        // }
        // NEW: drop GREEN if this robot is too far from anchor robot
        // if (!anchorRobotDistOK(r)) return false;
        if (!goalDedupOK(cand, r)) return false;
        if (checkIRCollisionSnap(r, cand)) return false;
        if (!pairwiseOKWithCurrentGreens(r, cand)) return false;
        if (!isLineFreeOnMap(cand, {snap.poses[r].position.x, snap.poses[r].position.y})) return false;
        if (!isLineFreeOnMap(cand, {snap.poses[r_anchor].position.x, snap.poses[r_anchor].position.y})) return false;
        if (!isLineFreeOnMap({snap.poses[r].position.x, snap.poses[r].position.y}, {snap.poses[r_anchor].position.x, snap.poses[r_anchor].position.y})) return false;
        return true;
    };

    // -----------------------------------------
    // Step G3 (NEW): filter GREENS by anchor-relative deviation (keep anchor)
    // Rule:
    //  - Keep anchor as "reference"
    //  - For other GREEN candidates, sort by "how far it deviates from anchor" (worst first)
    //    deviation score = wD * dist(t_r, t_anchor) + wA * angle(anchor_dir, r_dir)
    //  - Then greedily keep if canKeepGreen(r, cand) passes w.r.t current kept GREEN set
    //  - LoS remains pairwise between GREEN targets (handled inside canKeepGreen)
    // -----------------------------------------
    {
        // anchor target & direction reference
        const Vec2 t_anchor = green_targets[r_anchor];

        // If anchor itself is invalid, we cannot use anchor-relative filtering;
        // in that case, we just skip filtering order change (or you can fallback to robot-target distance).
        const bool anchor_valid = isFiniteVec2(t_anchor) &&
                                !isPermBlocked(t_anchor) &&
                                !isRobotForbidden(r_anchor, t_anchor);

        // weights (can be params, keep simple for now)
        const double wD = 1.0;   // distance-to-anchor-target weight
        const double wA = 0.5;   // angle-to-anchor-direction weight (deg)

        std::pair<double,double> anchor_dir{0.0, 0.0};
        if (anchor_valid) {
            anchor_dir = unitDirFromPoseToTarget(r_anchor, t_anchor);
        }

        std::vector<int> order;
        order.reserve(num_robot_);
        for (int r = 0; r < num_robot_; ++r) {
            if (!is_green[r]) continue;
            if (r == r_anchor) continue;
            if (retry_guard_valid && r == retry_guard_robot) continue;
            order.push_back(r);
        }

        auto deviationScore = [&](int r)->double {
            if (!anchor_valid) {
                // fallback: prioritize dropping those with larger robot->target distance
                return dist(poseXY(snap.poses[r]), green_targets[r]);
            }
            const Vec2 tr = green_targets[r];
            if (!isFiniteVec2(tr)) return 1e9;
            const double d = dist(tr, t_anchor);
            const auto rdir = unitDirFromPoseToTarget(r, tr);
            const double ang = angleDegBetween(anchor_dir, rdir);
            return wD * d + wA * ang;
        };

        // worst first (bigger deviation => earlier to be dropped)
        std::sort(order.begin(), order.end(), [&](int a, int b){
            return deviationScore(a) > deviationScore(b);
        });

        for (int r : order) {
            Vec2 cand = green_targets[r];

            // temporarily remove itself to evaluate cleanly against current kept greens
            is_green[r] = false;
            green_targets[r] = NaN2();

            if (canKeepGreen(r, cand)) {
                green_targets[r] = cand;
                is_green[r] = true;
            } else {
                // dropped -> ORANGE later
            }
        }

        // Anchor final validity: only basic validity check here (pairwise conflicts handled in Step G4)
        if (!isFiniteVec2(green_targets[r_anchor]) ||
            isPermBlocked(green_targets[r_anchor]) ||
            isRobotForbidden(r_anchor, green_targets[r_anchor]))
        {
            is_green[r_anchor] = false;
            green_targets[r_anchor] = NaN2();
        } else {
            is_green[r_anchor] = true;
        }
    }

    // -----------------------------------------
    // Step G4: enforce "each GREEN target executed by nearest robot" (simple repair)
    // -----------------------------------------
    auto nearestRobotToPoint = [&](const Vec2& p)->int {
        int best = -1;
        double bd = std::numeric_limits<double>::infinity();
        for (int r = 0; r < num_robot_; ++r) {
            Vec2 pr = poseXY(snap.poses[r]);
            double d = dist(pr, p);
            if (d < bd) { bd = d; best = r; }
        }
        return best;
    };

    auto canKeepGreenWithGivenSet = [&](int r, const Vec2& cand,
                                    const std::vector<Vec2>& tgt,
                                    const std::vector<bool>& isg)->bool {
        if (snap.emerg_stop[r] == true) return false;
        if (!isFiniteVec2(cand)) return false;
        if (isPermBlocked(cand)) return false;
        if (checkIRCollisionSnap(r, cand)) return false;
        if (isRobotForbidden(r, cand)) return false;
        if (!isLineFreeOnMap(cand, {snap.poses[r].position.x, snap.poses[r].position.y})) return false;   
        if (!isLineFreeOnMap(cand, {snap.poses[r_anchor].position.x, snap.poses[r_anchor].position.y})) return false;  
        if (!isLineFreeOnMap({snap.poses[r].position.x, snap.poses[r].position.y}, {snap.poses[r_anchor].position.x, snap.poses[r_anchor].position.y})) return false;
        // NEW: same anchor-distance gate
        // if (!anchorRobotDistOK(r)) return false;

        // goal dedup
        for (int g = 0; g < num_robot_; ++g) {
            if (g == r) continue;
            if (!isg[g]) continue;
            if (!isFiniteVec2(tgt[g])) continue;
            if (dist(cand, tgt[g]) < goal_dedup_eps_) return false;
        }

        // pairwise constraints ...
        for (int g = 0; g < num_robot_; ++g) {
            if (g == r) continue;
            if (!isg[g]) continue;
            if (!isFiniteVec2(tgt[g])) continue;

            if (!directionOK(r, cand, g, tgt[g])) return false;
            if (!directionOK(g, tgt[g], r, cand)) return false;

            const double d = dist(cand, tgt[g]);
            if (d < MIN_TGT_DIST || d > MAX_TGT_DIST) return false;
            if (!isLineFreeOnMap(cand, tgt[g])) return false;
            if (d < goal_dedup_eps_) return false;
        }
        return true;
    };
    auto nearestFeasibleRobotToPoint = [&](const Vec2& p,
                                        const std::vector<Vec2>& tgt,
                                        const std::vector<bool>& isg,
                                        int exclude = -1)->int
    {
        int best = -1;
        double bd = std::numeric_limits<double>::infinity();
        for (int r = 0; r < num_robot_; ++r) {
            if (r == exclude) continue;
            const Vec2 pr = poseXY(snap.poses[r]);
            const double d = dist(pr, p);
            if (!std::isfinite(d)) continue;

            // 构造一个临时集合：假设把 p 分配给 r（其余不变）
            std::vector<Vec2> tgt2 = tgt;
            std::vector<bool> isg2 = isg;
            tgt2[r] = p;
            isg2[r] = true;

            // 注意：如果 r 原本是绿且有别的目标，这里相当于“覆盖”它的目标；
            // G4 的语义就是在修复/调整所有权，所以允许覆盖，然后让后续逻辑决定是否交换/丢弃。
            if (!canKeepGreenWithGivenSet(r, p, tgt2, isg2)) continue;

            if (d < bd) { bd = d; best = r; }
        }
        return best;
    };
    const int MAX_FIX_IT = 10;
    for (int it = 0; it < MAX_FIX_IT; ++it) {
        bool changed = false;

        for (int r = 0; r < num_robot_; ++r) {
            if (!is_green[r]) continue;
            if (!isFiniteVec2(green_targets[r])) continue;

            const Vec2 tgt_r = green_targets[r];
            const bool is_retry_target = (retry_guard_valid && dist(tgt_r, retry_guard_target) < goal_dedup_eps_);

            // --- dynamic anchor target (avoid stale t_anchor/anchor_valid) ---
            Vec2 cur_anchor_tgt = NaN2();
            bool cur_anchor_valid = false;
            if (r_anchor >= 0 && r_anchor < num_robot_ && is_green[r_anchor] && isFiniteVec2(green_targets[r_anchor])) {
                cur_anchor_tgt = green_targets[r_anchor];
                cur_anchor_valid = true;
            }
            const bool is_anchor_target = (cur_anchor_valid && dist(tgt_r, cur_anchor_tgt) < goal_dedup_eps_);

            if (is_retry_target) {
                if (r != retry_guard_robot) {
                    is_green[r] = false;
                    green_targets[r] = NaN2();
                    is_green[retry_guard_robot] = true;
                    green_targets[retry_guard_robot] = retry_guard_target;
                    changed = true;
                }
                continue;
            }

            // --- pick nearest feasible owner instead of nearest-by-distance ---
            int q = nearestFeasibleRobotToPoint(tgt_r, green_targets, is_green, -1);
            if (q < 0) continue; // no feasible owner exists

            // if (anchor_lock_valid_this_round && r == r_anchor) {
            //     continue; // never move/swap anchor assignment under lock
            // }
            if (q == r) continue;

            // Case 1: q not green -> move tgt_r to q
            if (!is_green[q]) {
                std::vector<Vec2> tgt2 = green_targets;
                std::vector<bool> isg2 = is_green;

                tgt2[r] = NaN2(); isg2[r] = false;
                tgt2[q] = tgt_r;  isg2[q] = true;

                if (canKeepGreenWithGivenSet(q, tgt_r, tgt2, isg2)) {
                    green_targets = std::move(tgt2);
                    is_green      = std::move(isg2);

                    if (is_anchor_target) {
                        r_anchor = q;
                    }
                    changed = true;
                    break;
                } else {
                    // IMPORTANT: do NOT drop blindly if r itself is feasible; keep it.
                    // Here, since original state already had r->tgt_r feasible (it was GREEN),
                    // we simply do nothing instead of dropping it.
                    continue;
                }
            }

            // Case 2: q is green -> try swap (except anchor target special handling)
            {
                if (is_anchor_target) {
                    // re-assign anchor target to q and drop r
                    std::vector<Vec2> tgt2 = green_targets;
                    std::vector<bool> isg2 = is_green;

                    tgt2[r] = NaN2(); isg2[r] = false;
                    tgt2[q] = tgt_r;  isg2[q] = true;

                    if (canKeepGreenWithGivenSet(q, tgt_r, tgt2, isg2)) {
                        green_targets = std::move(tgt2);
                        is_green      = std::move(isg2);
                        r_anchor = q;
                    } else {
                        // anchor target can't be moved safely -> keep original assignment
                        continue;
                    }
                    changed = true;
                    break;
                }

                // normal swap
                std::vector<Vec2> tgt2 = green_targets;
                std::vector<bool> isg2 = is_green;

                Vec2 pr = tgt2[r];
                Vec2 pq = tgt2[q];
                tgt2[r] = pq;
                tgt2[q] = pr;

                if (canKeepGreenWithGivenSet(r, tgt2[r], tgt2, isg2) &&
                    canKeepGreenWithGivenSet(q, tgt2[q], tgt2, isg2)) {

                    green_targets = std::move(tgt2);
                    is_green      = std::move(isg2);
                    changed = true;
                    break;

                } else {
                    // swap not possible -> drop one with larger robot->target distance (with NaN guard)
                    Vec2 rr = poseXY(snap.poses[r]);
                    Vec2 rq = poseXY(snap.poses[q]);

                    double dr = std::numeric_limits<double>::infinity();
                    double dq = std::numeric_limits<double>::infinity();
                    if (isFiniteVec2(green_targets[r])) dr = dist(rr, green_targets[r]);
                    if (isFiniteVec2(green_targets[q])) dq = dist(rq, green_targets[q]);

                    int drop;
                    if (!std::isfinite(dr) && !std::isfinite(dq)) drop = r;
                    else if (!std::isfinite(dr)) drop = r;
                    else if (!std::isfinite(dq)) drop = q;
                    else drop = (dr >= dq) ? r : q;

                    green_targets[drop] = NaN2();
                    is_green[drop] = false;
                    changed = true;
                    break;
                }
            }
        }

        if (!changed) break;
    }
    // const int MAX_FIX_IT = 10;
    // for (int it = 0; it < MAX_FIX_IT; ++it) {
    //     bool changed = false;

    //     for (int r = 0; r < num_robot_; ++r) {
    //         if (!is_green[r]) continue;
    //         if (!isFiniteVec2(green_targets[r])) continue;

    //         const Vec2 tgt_r = green_targets[r];
    //         const bool is_retry_target = (retry_guard_valid && dist(tgt_r, retry_guard_target) < goal_dedup_eps_);

    //         if (is_retry_target) {
    //             // 1) 如果当前 owner 不是 retry_guard_robot，强制把它改回 retry_guard_robot
    //             if (r != retry_guard_robot) {
    //                 // 把 r 的 green 清掉（它会变橙），retry_robot 保留 retry_target
    //                 is_green[r] = false;
    //                 green_targets[r] = NaN2();
    //                 is_green[retry_guard_robot] = true;
    //                 green_targets[retry_guard_robot] = retry_guard_target;
    //                 changed = true;
    //             }
    //             // 2) 绝不再对这个 r 进行 swap / move
    //             continue;
    //         }
    //         int q = nearestRobotToPoint(tgt_r);
    //         if (anchor_lock_valid_this_round && r == r_anchor) {
    //             continue; // never move/swap anchor assignment under lock
    //         }
    //         if (q == r || q < 0) continue;

    //         // ---- NEW: if this target is the anchor target, keep target fixed but allow owner update ----
    //         const bool is_anchor_target = (anchor_valid && dist(tgt_r, t_anchor) < goal_dedup_eps_);

    //         // Case 1: q is not green -> move target to q, drop r
    //         if (!is_green[q]) {
    //             std::vector<Vec2> tgt2 = green_targets;
    //             std::vector<bool> isg2 = is_green;

    //             tgt2[r] = NaN2(); isg2[r] = false;
    //             tgt2[q] = tgt_r;  isg2[q] = true;

    //             if (canKeepGreenWithGivenSet(q, tgt_r, tgt2, isg2)) {
    //                 green_targets = std::move(tgt2);
    //                 is_green      = std::move(isg2);

    //                 // if anchor target moved, update r_anchor owner
    //                 if (is_anchor_target) {
    //                     r_anchor = q; // owner updated
    //                 }

    //                 changed = true;
    //                 break;
    //             } else {
    //                 // cannot move -> drop r to ORANGE
    //                 green_targets[r] = NaN2();
    //                 is_green[r] = false;
    //                 changed = true;
    //                 break;
    //             }
    //         }

    //         // Case 2: both green -> try swap (but keep anchor target fixed in place if you want)
    //         {
    //             // If you want anchor target fixed, forbid swapping it away:
    //             if (is_anchor_target) {
    //                 // Try just re-assign anchor target to its nearest owner (q) and drop current r
    //                 std::vector<Vec2> tgt2 = green_targets;
    //                 std::vector<bool> isg2 = is_green;

    //                 tgt2[r] = NaN2(); isg2[r] = false;
    //                 tgt2[q] = tgt_r;  isg2[q] = true;

    //                 if (canKeepGreenWithGivenSet(q, tgt_r, tgt2, isg2)) {
    //                     green_targets = std::move(tgt2);
    //                     is_green      = std::move(isg2);
    //                     r_anchor = q;
    //                 } else {
    //                     green_targets[r] = NaN2();
    //                     is_green[r] = false;
    //                 }
    //                 changed = true;
    //                 break;
    //             }

    //             // otherwise normal swap
    //             std::vector<Vec2> tgt2 = green_targets;
    //             std::vector<bool> isg2 = is_green;

    //             Vec2 pr = tgt2[r];
    //             Vec2 pq = tgt2[q];
    //             tgt2[r] = pq;
    //             tgt2[q] = pr;

    //             if (canKeepGreenWithGivenSet(r, tgt2[r], tgt2, isg2) &&
    //                 canKeepGreenWithGivenSet(q, tgt2[q], tgt2, isg2)) {

    //                 green_targets = std::move(tgt2);
    //                 is_green      = std::move(isg2);
    //                 changed = true;
    //                 break;

    //             } else {
    //                 // swap not possible -> drop the one farther from its assigned target
    //                 Vec2 rr = poseXY(snap.poses[r]);
    //                 Vec2 rq = poseXY(snap.poses[q]);
    //                 double dr = dist(rr, green_targets[r]);
    //                 double dq = dist(rq, green_targets[q]);
    //                 int drop = (dr >= dq) ? r : q;

    //                 green_targets[drop] = NaN2();
    //                 is_green[drop] = false;
    //                 changed = true;
    //                 break;
    //             }
    //         }
    //     }

    //     if (!changed) break;
    // }

    // -----------------------------------------
    // Retry-mode: keep ONLY retry_robot_ as GREEN, others forced to ORANGE
    // (so Step O will generate their orange targets normally, NOT stay)
    // -----------------------------------------
    if (retry_active_ && retry_robot_ >= 0 && retry_robot_ < num_robot_ && isFiniteVec2(retry_target_)) {
        for (int r = 0; r < num_robot_; ++r) {
            if (r == retry_robot_) continue;
            is_green[r] = false;
            green_targets[r] = NaN2();
        }
        // ensure retry robot remains green
        is_green[retry_robot_] = true;
        green_targets[retry_robot_] = retry_target_;
    }

    // -----------------------------------------
    // Step G5 (REWRITE): STUCK -> immediate takeover by "most idle" robot,
    // then enter retry mode (single robot keeps trying until success).
    // - No stuck_list
    // - No switching multiple robots
    // - No retry-frontier list
    // - Original stuck robot adds forbid (so it won't get it again)
    // -----------------------------------------
    {
        auto pickMostIdleRobot = [&](int exclude_robot, const std::vector<bool>& is_green_now)->int {
                // “最空闲”=距离最近障碍物最远（clearance 最大）
                // 优先从非 GREEN（橙色/空闲）的机器人里选；如果都 GREEN，就退化为全体里选（除 exclude）
                if (!MapReceived()) {
                    // 没地图时没法算 clearance：退化为离队友最远（或离 target 最近也行）
                    int best = -1;
                    double best_score = -1.0;
                    for (int r = 0; r < num_robot_; ++r) {
                        if (r == exclude_robot) continue;
                        double score = 0.0;
                        for (int k = 0; k < num_robot_; ++k) {
                            if (k == r) continue;
                            score += std::hypot(snap.poses[r].position.x - snap.poses[k].position.x,
                                                snap.poses[r].position.y - snap.poses[k].position.y);
                        }
                        if (score > best_score) { best_score = score; best = r; }
                    }
                    return best;
                }

                auto clearanceOf = [&](int r)->double {
                    nav_msgs::OccupancyGrid m;
                    {
                        std::lock_guard<std::mutex> lk(map_mtx_);
                        if (!has_map_) return 0.0;
                        m = map_;
                    }
                    return distanceToNearestObstacle(m,
                                                    snap.poses[r].position.x,
                                                    snap.poses[r].position.y,
                                                    idle_clearance_search_radius_);
                };

                // 1) 先找非绿机器人（空闲）
                int best = -1;
                double best_clear = -1.0;
                for (int r = 0; r < num_robot_; ++r) {
                    if (r == exclude_robot) continue;
                    if (r < (int)is_green_now.size() && is_green_now[r]) continue; // 只从“非 GREEN”里选
                    const double c = clearanceOf(r);
                    if (c > best_clear) { best_clear = c; best = r; }
                }
                if (best != -1) return best;

                // 2) 如果全是绿，就从全体里选（除 exclude）
                for (int r = 0; r < num_robot_; ++r) {
                    if (r == exclude_robot) continue;
                    const double c = clearanceOf(r);
                    if (c > best_clear) { best_clear = c; best = r; }
                }
                return best;
            };

        auto isStuckNow = [&](int r, const Vec2& tgt)->bool {
            const Vec2 cur_pose = poseXY(snap.poses[r]);
            const double cur_dist = dist(cur_pose, tgt);

            if (cur_dist <= reach_radius_) {
                stuck_ct_[r] = 0;
                last_pose_xy_[r] = cur_pose;
                last_dist_to_goal_[r] = cur_dist;
                return false;
            }

            if (!isFiniteVec2(last_pose_xy_[r]) || !std::isfinite(last_dist_to_goal_[r])) {
                stuck_ct_[r] = 0;
                last_pose_xy_[r] = cur_pose;
                last_dist_to_goal_[r] = cur_dist;
                return false;
            }

            const double moved = dist(cur_pose, last_pose_xy_[r]);
            const double prog  = last_dist_to_goal_[r] - cur_dist; // progress>0 means closer

            const bool no_move = (moved < stuck_move_eps_);
            const bool no_prog = (prog  < stuck_prog_eps_);

            if (no_move || no_prog) stuck_ct_[r] += 1;
            else                    stuck_ct_[r]  = 0;

            last_pose_xy_[r] = cur_pose;
            last_dist_to_goal_[r] = cur_dist;

            return stuck_ct_[r] >= stuck_cycles_;
            return false;
        };

        // 如果已经在 retry 模式，不再对 retry_robot_ 做 stuck 切换（你要求“就算 stuck 也不切换”）
        for (int r = 0; r < num_robot_; ++r) {
            if (!is_green[r]) continue;
            if (!isFiniteVec2(green_targets[r])) continue;
            if (retry_guard_valid && r == retry_guard_robot) continue;
            if (retry_active_ && r == retry_robot_) continue;

            if (!isStuckNow(r, green_targets[r])) continue;

            const Vec2 stuck_target = green_targets[r];

            // 1) 原 stuck robot 加 forbid（但不加任何其它列表）
            addRobotForbidden(r, stuck_target);

            // 2) 原 robot 立刻降为 ORANGE（释放它的绿目标）
            is_green[r] = false;
            green_targets[r] = NaN2();

            // 3) 选择“最空闲机器人”来接管（优先选当前非绿机器人）
            int q = pickMostIdleRobot(r, is_green);

            if (q < 0 || q >= num_robot_) {
                ROS_WARN("\033[38;5;208m STUCK r%d but no takeover robot found -> keep all normal (target dropped this round)\033[0m", r+1);
                break; // 本轮不进入 retry
            }

            // 4) 进入 retry 模式：只让 q 一直尝试这个 target，直到 reach
            retry_active_ = true;
            retry_robot_  = q;
            retry_target_ = stuck_target;

            // 关键：你要“无论如何都让 q 试这个 target”
            // 所以如果 q 之前也 forbid 过这个 target，我们这里强制清掉 q 的 forbid 邻域（只针对该 target）
            eraseForbiddenNearRobot(q, stuck_target, robot_forbidden_merge_eps_);

            ROS_WARN("\033[41;37m STUCK takeover: r%d stuck on (%.2f,%.2f) -> retry_robot r%d (most-idle)\033[0m",
                    r+1, stuck_target.first, stuck_target.second, q+1);

            // 注意：一旦进入 retry 模式，本轮后续照旧走，但下一轮会被“retry mode short-circuit”提前 return
            break; // 一次只处理一个 stuck 事件，避免同时触发多个进入 retry
        }
    }

    // -----------------------------------------
    // Step O: fill ORANGE for non-green robots (reuse your neighbor PC logic)
    // -----------------------------------------
    std::vector<bool> is_temp(num_robot_, true); // true => ORANGE
    for (int r = 0; r < num_robot_; ++r) {
        if (is_green[r] && isFiniteVec2(green_targets[r])) {
            out.targets_xy[r] = green_targets[r];
            is_temp[r] = false;
        }
    }

    for (int r = 0; r < num_robot_; ++r) {
        if (!is_temp[r]) continue; // already green

        Vec2 tmp = poseXY(snap.poses[r]);
        std::vector<int> nei = (snap.neighbors.size() == (size_t)num_robot_) ? snap.neighbors[r]
                                                                             : std::vector<int>{};

        while (!nei.empty()) {
            int ref = selectNeighborRefRobot(r, nei, out.targets_xy, is_temp);
            nei.erase(std::remove(nei.begin(), nei.end(), ref), nei.end());
            if (ref < 0 || ref >= num_robot_ || ref == r) continue;

            int ego = r;
            int rr = ref;
            if (EvaluateNeiboringTarget(ego, rr, tmp, out.targets_xy, snap.emerg_stop[ego])) break;
        }
        out.targets_xy[r] = tmp;
        is_temp[r] = true;
    }

    // -----------------------------------------
    // NEW: All-ORANGE persistence counter and gradual relax of perm_block_frontiers_
    // Trigger only when:
    //   1) NOT paused_this_round (avoid confusing with anchor-jump wait)
    //   2) ALL robots are ORANGE after normal assignment pipeline
    //   3) all_orange_ct_ >= M
    // Action:
    //   remove ONE perm-blocked point per cycle (nearest to current team center)
    // -----------------------------------------
    {
        bool any_green_now = false;
        for (int r = 0; r < num_robot_; ++r) {
            if (!is_temp[r]) { any_green_now = true; break; }
        }

        // Count only when not in pause mode
        if (!paused_this_round && !any_green_now) {
            all_orange_ct_++;
        } else {
            all_orange_ct_ = 0;
        }

        if (!paused_this_round && !any_green_now && all_orange_ct_ >= all_orange_relax_M_) {
            if (!stuck_retry_frontiers_.empty()) {
                Vec2 center{0.0, 0.0};
                for (int r = 0; r < num_robot_; ++r) {
                    center.first  += snap.poses[r].position.x;
                    center.second += snap.poses[r].position.y;
                }
                center.first  /= (double)num_robot_;
                center.second /= (double)num_robot_;

                int best_i = -1;
                double best_d = std::numeric_limits<double>::infinity();
                for (int i = 0; i < (int)stuck_retry_frontiers_.size(); ++i) {
                    double d = dist(center, stuck_retry_frontiers_[i]);
                    if (d < best_d) { best_d = d; best_i = i; }
                }
  
                if (best_i >= 0) {
                    Vec2 removed = stuck_retry_frontiers_[best_i];
                    stuck_retry_frontiers_.erase(stuck_retry_frontiers_.begin() + best_i);

                    ROS_WARN("\033[38;5;208m All-ORANGE persisted %d cycles (M=%d): erase retry-frontier (%.2f,%.2f), d=%.2f, retry_size=%zu \033[0m",
                            all_orange_ct_, all_orange_relax_M_,
                            removed.first, removed.second, best_d, stuck_retry_frontiers_.size());

                    const double eps = std::max(robot_forbidden_merge_eps_, perm_block_merge_eps_);
                    eraseForbiddenNearAllRobots(removed, eps);
                    ROS_WARN("\033[38;5;208m Also erased robot_forbidden_targets_ near erased retry-frontier (eps=%.2f)\033[0m", eps);
                }
            } else {
                ROS_WARN_THROTTLE(1.0, "All-ORANGE persisted but stuck_retry_frontiers_ is empty (perm_block_frontiers_ will NOT be erased).");
            }
        }
    }

    // -----------------------------------------
    // Publish markers + target type + print
    // -----------------------------------------
    {
        visualization_msgs::Marker mk_assigned =
            makeSphereListMarker("assigned_targets", 0, makeColor(0.0f, 1.0f, 0.0f, 1.0f));
        visualization_msgs::Marker mk_temp =
            makeSphereListMarker("temp_targets", 0, makeColor(1.0f, 0.55f, 0.0f, 1.0f));

        for (int r = 0; r < num_robot_; ++r) {
            geometry_msgs::Point p;
            p.x = out.targets_xy[r].first;
            p.y = out.targets_xy[r].second;
            p.z = 0.05;
            if (is_temp[r]) mk_temp.points.push_back(p);
            else            mk_assigned.points.push_back(p);
        }

        pub_assigned_targets_marker_.publish(mk_assigned);
        pub_temp_targets_marker_.publish(mk_temp);

        std_msgs::Bool msg;
        for (int r = 0; r < num_robot_; ++r) {
            msg.data = (!is_temp[r]);
            pub_targets_type_[r].publish(msg);
        }
        std::ostringstream oss;
        oss << "Assigned targets:";
        for (int r = 0; r < num_robot_; ++r) {
            const auto& t = out.targets_xy[r];
            if (is_temp[r]) {
                oss << " " << "\033[38;5;208m"
                    << "r" << (r + 1) << "->(" << t.first << "," << t.second << ")"
                    << "\033[0m";
            } else {
                oss << " " << "\033[32m"
                    << "r" << (r + 1) << "->(" << t.first << "," << t.second << ")"
                    << "\033[0m";
            }
        }
        ROS_INFO_STREAM(oss.str());
    }

    // -----------------------------------------
    // Publish goals with simple gating + update memory
    // -----------------------------------------
    auto shouldPublish = [&](int r)->bool {
        const Vec2& newg = out.targets_xy[r];
        if (!isFiniteVec2(newg)) return false;
        // if (!has_published_goal_[r]) return true;
        // if (last_published_is_temp_[r] != is_temp[r]) return true;
        // return dist(newg, last_published_goal_[r]) > 0.20;
    };

    std::vector<bool> published_this_round(num_robot_, false);

    for (int r = 0; r < num_robot_; ++r) {
        // if (shouldPublish(r)) {
            publishMoveBaseSimpleGoal(r, out.targets_xy[r], snap.poses[r]);
            last_published_goal_[r] = out.targets_xy[r];
            has_published_goal_[r] = true;
            // last_published_is_temp_[r] = is_temp[r];
            published_this_round[r] = true;
        // }
    }

    {
        // std::lock_guard<std::mutex> lock(mtx_);
        for (int r = 0; r < num_robot_; ++r) {
            if (!isFiniteVec2(out.targets_xy[r])) continue;

            last_targets_xy_[r] = out.targets_xy[r];
            last_is_temp_[r] = is_temp[r];
            has_target_[r] = true;

            if (!is_temp[r]) {
                last_green_targets_xy_[r] = out.targets_xy[r];
                has_green_target_[r] = true;
            }
        }
    }
    // -----------------------------------------
    // NEW: remember ONLY reached GREEN targets
    // -----------------------------------------
    {
        for (int r = 0; r < num_robot_; ++r) {
            if (is_temp[r]) continue;                  // only GREEN
            const Vec2& g = out.targets_xy[r];
            if (!isFiniteVec2(g)) continue;

            // reached (use your existing helper; it uses poses_)
            if (ReachedTarget(r, g, reach_radius_)) {
                rememberAnchorTarget(g);
            }
        }
    }
    // -----------------------------------------
    // Record "this round published GREEN targets" as reference for next anchor jump guard
    // -----------------------------------------
    {
        Vec2 ref{0.0, 0.0};
        Vec2 f_ref{0.0, 0.0};
        last_round_green_dir_ = {0.0, 0.0};
        int cnt = 0;

        Vec2 robots_center{0.0, 0.0};
        for (int r = 0; r < num_robot_; ++r) {
            robots_center.first  += snap.poses[r].position.x;
            robots_center.second += snap.poses[r].position.y;
        }
        robots_center.first /= (double)num_robot_;
        robots_center.second/= (double)num_robot_;

        for (int r = 0; r < num_robot_; ++r) {
            if (!published_this_round[r]) continue;   // only those actually published
            if (is_temp[r]) continue;                 // only GREEN
            if (!isFiniteVec2(out.targets_xy[r])) continue;
            f_ref.first = out.targets_xy[r].first;
            f_ref.second = out.targets_xy[r].second;
            ref.first = snap.poses[r].position.x;
            ref.second = snap.poses[r].position.y;
            last_round_green_dir_.first += unitDir(ref, f_ref).first;
            last_round_green_dir_.second += unitDir(ref, f_ref).second;
            cnt++;
        }
        
        if (cnt > 0) {
            // update reference only when we have at least one published GREEN
            last_round_green_ref_ = f_ref;
            has_last_round_green_ref_ = true;

            // direction: from ref to fref 
            last_round_green_dir_.first /= cnt;
            last_round_green_dir_.second /= cnt;
            has_last_round_green_dir_ = true;
            allow_anchor_jump_ = false;
        }
    }
    // publish record traj info
    for (int r = 0; r < num_robot_; ++r) {
        record_traj_msg.data[r*5 + 0] = record_traj_info_[r][0];
        record_traj_msg.data[r*5 + 1] = record_traj_info_[r][1];
        record_traj_msg.data[r*5 + 2] = record_traj_info_[r][2];
        record_traj_msg.data[r*5 + 3] = record_traj_info_[r][3];
        record_traj_msg.data[r*5 + 4] = record_traj_info_[r][4];
    }
    record_traj_msg.data[num_robot_ * 5] = r_anchor;
    pub_record_traj_info_.publish(record_traj_msg);
    return out;
}

// ============================================================
// main (same as you had)
// ============================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_assigner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    FrontierAssigner assigner(nh, pnh);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (!assigner.allPosesReady() || !assigner.allConvexHullsReady() ||
            !assigner.allFrontiersReady() || !assigner.GraphReceived() ||
            // !assigner.allPCReady() || 
            !assigner.MapReceived())
        {
            rate.sleep();
            continue;
        }

        ros::Time t_start = ros::Time::now();

        auto res = assigner.assignCloseFrontiers();

        ros::Time t_end = ros::Time::now();
        double dt_ms = (t_end - t_start).toSec() * 1000.0;

        ROS_INFO_STREAM("assignCloseFrontiers() took " << dt_ms << " ms");

        if (res.navigation_done) break;

        rate.sleep();
    }

    return 0;
}