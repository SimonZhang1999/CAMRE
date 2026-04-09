#include "FrontierAssigner.hpp"

#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <limits>
#include <set>
#include <sstream>

// ============================================================
// FrontierAssigner ctor
// ============================================================

FrontierAssigner::FrontierAssigner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
{
    scp_ = std::make_shared<star_convex_handle::StarConvexHandler>(nh_);

    pub_assigned_targets_marker_ = nh_.advertise<visualization_msgs::Marker>("assigned_targets_marker", 1);
    pub_temp_targets_marker_     = nh_.advertise<visualization_msgs::Marker>("temp_targets_marker", 1);

    pnh.param<std::string>("global_frame", global_frame_, std::string("map"));
    pnh.param("target_marker_scale", target_marker_scale_, 0.35);
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

    pnh.param("stuck_move_eps", stuck_move_eps_, 0.10);
    pnh.param("stuck_prog_eps", stuck_prog_eps_, 0.05);
    pnh.param("stuck_cycles", stuck_cycles_, 10);
    pnh.param("block_merge_eps", block_merge_eps_, 0.30);

    nh.param("jump_guard_dist_th", jump_guard_dist_th_, 6.0);
    nh.param("jump_guard_yaw_th_deg", jump_guard_yaw_th_, 60.0);
    nh.param("jump_guard_max_ct", jump_guard_max_ct_, 3);
    pnh.param("perm_block_merge_eps", perm_block_merge_eps_, 0.30);
    pub_targets_type_.resize(num_robot_);
    for (int i = 0; i < num_robot_; i++) {
        pub_targets_type_[i] = nh_.advertise<std_msgs::Bool>("/robot_"+std::to_string(i+1)+"/target_type", 1);
    }
    if (num_robot_ <= 0) {
        ROS_FATAL("FrontierAssigner: param '~num_robot' must be > 0");
        throw std::runtime_error("invalid num_robot");
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
    last_green_targets_xy_.assign(num_robot_, {std::numeric_limits<double>::quiet_NaN(),
                                            std::numeric_limits<double>::quiet_NaN()});
    has_green_target_.assign(num_robot_, false);
    poses_.resize(num_robot_);
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

    last_targets_xy_.assign(num_robot_, {std::numeric_limits<double>::quiet_NaN(),
                                        std::numeric_limits<double>::quiet_NaN()});
    has_target_.assign(num_robot_, false);
    last_is_temp_.assign(num_robot_, true); // default ORANGE
    last_pose_xy_.assign(num_robot_, {std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN()});
    last_dist_to_goal_.assign(num_robot_, std::numeric_limits<double>::infinity());
    stuck_ct_.assign(num_robot_, 0);

    // publishing cache
    last_published_goal_.assign(num_robot_, {std::numeric_limits<double>::quiet_NaN(),
                                            std::numeric_limits<double>::quiet_NaN()});
    has_published_goal_.assign(num_robot_, false);
    last_published_is_temp_.assign(num_robot_, true);

    // Subscribers
    edges_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/mst_graph/edges", 1,
                                                           &FrontierAssigner::edgesCb, this);
    sub_planning_map_ = nh.subscribe("/planning_map", 1, &FrontierAssigner::planningMapCb, this);

    for (int i = 0; i < num_robot_; ++i) {
        pose_subs_[i] = nh_.subscribe<nav_msgs::Odometry>(
            "/robot_" + std::to_string(i + 1) + "/RosAria/odom", 1,
            boost::bind(&FrontierAssigner::poseCb, this, _1, i));

        frontier_subs_[i] = nh_.subscribe<cure_planner::PointArray>(
            frontier_topics_[i], 1,
            boost::bind(&FrontierAssigner::frontiersCb, this, _1, i));

        emergency_stop_subs_[i] = nh_.subscribe<std_msgs::Bool>(
            "/robot_" + std::to_string(i + 1) + "/planner/emergency_stop", 1,
            boost::bind(&FrontierAssigner::emergencystopCb, this, _1, i));

        subs_vertices_convexhull_[i] = nh_.subscribe<std_msgs::Float32MultiArray>(
            "/robot_" + std::to_string(i + 1) + "/vertices_convexhull", 1,
            boost::bind(&FrontierAssigner::convexhullsCb, this, _1, i));

        PC_subs_[i] = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/robot_" + std::to_string(i + 1) + "/NeighborPointcloud", 1,
            boost::bind(&FrontierAssigner::neiPcCb, this, _1, i));

        // publish move_base_simple/goal
        const std::string goal_topic = "/robot_" + std::to_string(i + 1) + "/move_base_simple/goal";
        goal_pubs_[i] = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    }

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
    try
    {
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
    }
    catch (const tf::TransformException& ex)
    {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    }
}

void FrontierAssigner::frontiersCb(const cure_planner::PointArrayConstPtr& msg, int robot_idx) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<Vec2> list;
    list.reserve(msg->points.size());
    for (const auto& p : msg->points) {
        list.emplace_back(p.x, p.y);
    }
    frontiers_by_robot_[robot_idx] = std::move(list);
    has_frontiers_[robot_idx] = true;
}

void FrontierAssigner::convexhullsCb(const std_msgs::Float32MultiArray::ConstPtr& msg, int idx) {
    std::lock_guard<std::mutex> lock(mtx_);

    // Check poses ready inside lock
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

    for (; it_x != it_x.end(); ++it_x, ++it_y) {
        tmp.emplace_back(*it_x, *it_y);
    }

    robots_neighbor_pc_[robot_idx] = std::move(tmp);
    has_pc_[robot_idx] = true;
}

void FrontierAssigner::edgesCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    have_graph_ = true;
    robot_neighbors_.assign(num_robot_, std::vector<int>{});

    const size_t n = msg->data.size();

    // format:
    //  pair:   [u,v,u,v,...] (1-based indices)
    //  triple: [u,v,w,u,v,w,...]
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

    // De-dup
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
    std::lock_guard<std::mutex> lock(mtx_);

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

// ============================================================
// Legacy helpers (kept as-is)
// ============================================================

bool FrontierAssigner::FrontiersFeasibility(const Vec2& frontier) const {
    std::lock_guard<std::mutex> lock(mtx_);
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
    std::lock_guard<std::mutex> lock(mtx_);
    return mergeAllFrontiersLocked();
}

// ============================================================
// Legacy functions used by older pipeline (kept)
// ============================================================

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
            if (std::hypot(x - target_set[i].first, y - target_set[i].second) < 1.8) {
                return true;
            }
        }
    }
    return false;
}

bool FrontierAssigner::CheckIRCollision(const int& ego_idx, const double& x, const double& y) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < (int)poses_.size(); i++) {
        if (i == ego_idx) continue;
        if (std::hypot(poses_[i].position.x - x, poses_[i].position.y - y) < 1.8) {
            return true;
        }
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
    std::lock_guard<std::mutex> lock(mtx_);
    const double dx = poses_[r].position.x - target.first;
    const double dy = poses_[r].position.y - target.second;
    return std::hypot(dx, dy) <= radius;
}

bool FrontierAssigner::FrontierDirectionCheck(int r_idx,
                                              const Vec2& candidate,
                                              int g_idx,
                                              const Vec2& g_target) const
{
    std::lock_guard<std::mutex> lock(mtx_);
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

double FrontierAssigner::computeCellCost(int& ego_idx, int& ri_idx, const Vec2& candidate_cell) const {
    std::lock_guard<std::mutex> lock(mtx_);

    auto yaw_ref  = yawFromTo(poses_[ego_idx], {poses_[ri_idx].position.x, poses_[ri_idx].position.y});
    auto yaw_this = yawFromTo(poses_[ego_idx], candidate_cell);

    double d = std::atan2(std::sin(yaw_ref - yaw_this), std::cos(yaw_ref - yaw_this));
    double delta_yaw = std::fabs(d);

    if (delta_yaw > M_PI / 2) return 1e5;

    const auto& hull = robots_convexhull_[ri_idx];
    if (hull.empty()) return std::numeric_limits<double>::infinity();

    // double num = scp_->starConvexConstraint({candidate_cell.first, candidate_cell.second},
    //                                        {poses_[ri_idx].position.x, poses_[ri_idx].position.y},
    //                                        hull, false)[0];
    // double den = scp_->sensitivityCost({candidate_cell.first, candidate_cell.second},
    //                                    {poses_[ri_idx].position.x, poses_[ri_idx].position.y},
    //                                    hull);

    // if (!std::isfinite(num) || !std::isfinite(den) || den < 1e-6) {
    //     return std::numeric_limits<double>::infinity();
    // }
    // return num / den;
}

double FrontierAssigner::computeLoSCost(int& ego_idx, int& ri_idx) const {
    std::lock_guard<std::mutex> lock(mtx_);
    const auto& hull = robots_convexhull_[ri_idx];
    if (hull.empty()) return std::numeric_limits<double>::infinity();
    return 0.0;
    // return scp_->starConvexConstraint({poses_[ego_idx].position.x, poses_[ego_idx].position.y},
    //                                  {poses_[ri_idx].position.x, poses_[ri_idx].position.y},
    //                                  hull);
}

bool FrontierAssigner::EvaluateNeiboringTarget(int& ego_idx, int& r_idx, Vec2& tmp,
                                               const std::vector<Vec2>& target_set) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (r_idx < 0 || r_idx >= num_robot_) return false;

    if (std::hypot(poses_[ego_idx].position.x - poses_[r_idx].position.x,
                   poses_[ego_idx].position.y - poses_[r_idx].position.y) <= 1.8) {
        tmp = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};
        return true;
    }

    const auto& pc = robots_neighbor_pc_[r_idx];
    if (pc.empty()) {
        tmp = {poses_[ego_idx].position.x, poses_[ego_idx].position.y};
        return false;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    Vec2 best_pt = {poses_[r_idx].position.x, poses_[r_idx].position.y};

    for (const auto& r : pc) {
        if (CheckExistingTargets(ego_idx, r.x(), r.y(), target_set)) continue;
        if (CheckIRCollision(ego_idx, r.x(), r.y())) continue;

        Vec2 cand{r.x(), r.y()};
        double c = computeCellCost(ego_idx, r_idx, cand);
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
    std::lock_guard<std::mutex> lock(mtx_);

    const Vec2 r_pos = poseXY(poses_[r]);

    // 1) among neighbors with GREEN targets, choose the NEAREST such target
    int best_idx = -1;
    double best_d = std::numeric_limits<double>::infinity();

    for (int n : nei_idx_set) {
        if (n < 0 || n >= num_robot_ || n == r) continue;
        if (!isFiniteVec2(assigned_targets_xy[n]) || is_temp[n]) continue;

        const double d = dist(r_pos, assigned_targets_xy[n]);
        if (d < best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    if (best_idx != -1) return best_idx;

    // 2) otherwise choose the NEAREST neighbor robot
    best_d = std::numeric_limits<double>::infinity();
    for (int n : nei_idx_set) {
        if (n < 0 || n >= num_robot_ || n == r) continue;
        const Vec2 n_pos = poseXY(poses_[n]);
        const double d = dist(r_pos, n_pos);
        if (d < best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    if (best_idx != -1) return best_idx;

    // 3) fallback global nearest (excluding self)
    best_d = std::numeric_limits<double>::infinity();
    for (int n = 0; n < num_robot_; ++n) {
        if (n == r) continue;
        const Vec2 n_pos = poseXY(poses_[n]);
        const double d = dist(r_pos, n_pos);
        if (d < best_d) {
            best_d = d;
            best_idx = n;
        }
    }
    return best_idx;
}

// ============================================================
// AssignCloseFrontiers
// ============================================================

FrontierAssigner::AssignResult FrontierAssigner::assignCloseFrontiers() {
    AssignResult out;
    out.leader_idx = -1;
    out.targets_xy.assign(num_robot_, {std::numeric_limits<double>::quiet_NaN(),
                                       std::numeric_limits<double>::quiet_NaN()});
    out.navigation_done = false;

    const Snapshot snap = getSnapshotLocked();

    if ((int)snap.poses.size() != num_robot_) {
        ROS_ERROR("assignCloseFrontiers: poses size %zu != num_robot %d", snap.poses.size(), num_robot_);
        return out;
    }

    struct Ctx {
        FrontierAssigner* self;
        const Snapshot& snap;
        AssignResult& out;

        std::vector<bool> is_temp;          // true: ORANGE, false: GREEN
        std::vector<bool> force_publish;    // force publish even if goal unchanged

        // merged UNIQUE frontiers for this round (cluster representative)
        std::vector<Vec2> frontiers;

        // bookkeeping
        std::set<int> assigned_robots;
        std::set<int> assigned_fids;            // occupied unique frontier id
        std::vector<int> green_set;             // robots that are GREEN
        std::vector<int> robot_assigned_fid;    // robot -> fid, -1 if not from current frontier
        std::vector<bool> dropped_green;  // 本轮被GlobalFix从GREEN drop 的机器人

        explicit Ctx(FrontierAssigner* s, const Snapshot& sn, AssignResult& o)
            : self(s), snap(sn), out(o)
        {
            is_temp.assign(self->num_robot_, true);
            force_publish.assign(self->num_robot_, false);
            green_set.reserve(self->num_robot_);
            robot_assigned_fid.assign(self->num_robot_, -1);
            dropped_green.assign(self->num_robot_, false);
        }

        static inline Vec2 NaN2() {
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};
        }

        Vec2 poseXY(int i) const { return {snap.poses[i].position.x, snap.poses[i].position.y}; }
        double dist2(const Vec2& a, const Vec2& b) const { return std::hypot(a.first - b.first, a.second - b.second); }

        double shortestAbsYawDiff(double a, double b) const {
            const double d = std::atan2(std::sin(a - b), std::cos(a - b));
            return std::fabs(d);
        }

        double yawFromTo(int from_idx, const Vec2& to) const {
            return std::atan2(to.second - snap.poses[from_idx].position.y,
                              to.first  - snap.poses[from_idx].position.x);
        }

        double yawOfPose(int i) const {
            const auto& q = snap.poses[i].orientation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return std::atan2(siny_cosp, cosy_cosp);
        }
        bool isPermBlocked(const Vec2& f) const {
            for (const auto& bf : self->perm_block_frontiers_) {
                if (dist2(f, bf) < self->perm_block_merge_eps_) return true;
            }
            return false;
        }

        void addPermBlocked(const Vec2& p) {
            if (!isFiniteVec2(p)) return;
            // 去重：近的就不重复加
            for (const auto& bf : self->perm_block_frontiers_) {
                if (dist2(p, bf) < self->perm_block_merge_eps_) return;
            }
            self->perm_block_frontiers_.push_back(p);
        }
        // ref forward direction:
        // prefer ref -> ref_target ONLY if ref is GREEN (avoid ORANGE polluting forward),
        // otherwise use ref pose yaw.
        std::pair<double,double> refForward(int ref) const {
            const Vec2 ref_xy = poseXY(ref);

            if (ref >= 0 && ref < self->num_robot_ &&
                !is_temp[ref] && isFiniteVec2(out.targets_xy[ref])) {
                const Vec2 rt = out.targets_xy[ref];
                double vx = rt.first  - ref_xy.first;
                double vy = rt.second - ref_xy.second;
                double n = std::hypot(vx, vy);
                if (std::isfinite(n) && n > 1e-6) return {vx / n, vy / n};
            }

            const double yaw = yawOfPose(ref);
            return {std::cos(yaw), std::sin(yaw)};
        }

        int frontSignWrtRef(int ref, const Vec2& p) const {
            const Vec2 ref_xy = poseXY(ref);
            const auto f = refForward(ref);
            const double vx = p.first  - ref_xy.first;
            const double vy = p.second - ref_xy.second;
            const double dot = vx * f.first + vy * f.second;
            const double eps = 1e-3;
            if (dot >= eps)  return +1;
            if (dot <= -eps) return -1;
            return 0;
        }

        bool checkIRCollision(int ego, const Vec2& p) const {
            for (int i = 0; i < self->num_robot_; ++i) {
                if (i == ego) continue;
                if (std::hypot(snap.poses[i].position.x - p.first,
                               snap.poses[i].position.y - p.second) < 1.8) {
                    return true;
                }
            }
            return false;
        }

        std::pair<double,double> dirUnitOrYaw(int idx, const Vec2& target) const {
            const Vec2 p = poseXY(idx);
            double vx = target.first  - p.first;
            double vy = target.second - p.second;
            double n  = std::hypot(vx, vy);

            if (std::isfinite(n) && n > 0.5) {
                return {vx / n, vy / n};
            }

            const double yaw = yawOfPose(idx);
            return {std::cos(yaw), std::sin(yaw)};
        }

        bool directionOK(int r_idx, const Vec2& candidate, int g_idx, const Vec2& g_target) const {
            if (!isFiniteVec2(candidate) || !isFiniteVec2(g_target)) return false;

            const auto vr = dirUnitOrYaw(r_idx, candidate);
            const auto vg = dirUnitOrYaw(g_idx, g_target);

            double c = vr.first * vg.first + vr.second * vg.second;
            if (!std::isfinite(c)) return false;
            c = std::max(-1.0, std::min(1.0, c));

            const double theta = std::acos(c);
            const double theta_max = self->max_heading_angle_deg_ * M_PI / 180.0;

            if (theta >= theta_max) {
                ROS_INFO("\033[40;36m Ori violated: r%d cand[%f,%f] vs g%d tgt[%f,%f], theta=%.1fdeg > th=%.1fdeg \033[0m",
                        r_idx+1, candidate.first, candidate.second,
                        g_idx+1, g_target.first, g_target.second,
                        theta * 180.0 / M_PI, theta_max * 180.0 / M_PI);
                return false;
            }
            return true;
        }

        bool feasibleDistToGreens(int ego, const Vec2& cand) const {
            for (int g : green_set) {
                if (g == ego) continue;
                if (!isFiniteVec2(out.targets_xy[g]) || is_temp[g]) continue;
                const double d = dist2(cand, out.targets_xy[g]);
                if (d < self->MIN_TGT_DIST || d > self->MAX_TGT_DIST) {
                    ROS_INFO("\033[40;36m Distance constraints are violated, invalid target:[%f, %f]\033[0m",
                             cand.first, cand.second);
                    return false;
                }
            }
            return true;
        }

        bool feasibleLoSToGreens(int ego, const Vec2& cand) const {
            for (int g : green_set) {
                if (g == ego) continue;
                if (!isFiniteVec2(out.targets_xy[g]) || is_temp[g]) continue;
                if (!self->isLineFreeOnMap(cand, out.targets_xy[g])) {
                    ROS_INFO("\033[40;36m LoS blocked by obstacle between GREEN targets -> invalid target: [%f, %f] \033[0m",
                             cand.first, cand.second);
                    return false;
                }
            }
            return true;
        }

        // IMPORTANT: also enforce goal de-dup (independent from MIN_TGT_DIST)
        bool goalDedupOK(const Vec2& cand) const {
            for (int g : green_set) {
                if (is_temp[g]) continue;
                if (!isFiniteVec2(out.targets_xy[g])) continue;
                if (dist2(cand, out.targets_xy[g]) < self->goal_dedup_eps_) return false;
            }
            return true;
        }

        bool canAssignGreen(int r, const Vec2& cand) const {
            if (isPermBlocked(cand)) return false;
            if (!isFiniteVec2(cand)) return false;

            // hard de-dup: avoid multiple robots selecting same/near goal
            if (!goalDedupOK(cand)) return false;

            // Direction constraint vs ALL existing GREEN targets
            for (int g : green_set) {
                if (g == r) continue;
                if (g < 0 || g >= self->num_robot_) continue;
                if (is_temp[g]) continue;
                if (!isFiniteVec2(out.targets_xy[g])) continue;
                if (!directionOK(r, cand, g, out.targets_xy[g])) return false;
            }

            if (!feasibleDistToGreens(r, cand)) return false;
            if (!feasibleLoSToGreens(r, cand))  return false;
            if (checkIRCollision(r, cand))      return false;
            return true;
        }

        void eraseBlockedIfNear(const Vec2& p) {
            self->block_frontiers_.erase(
                std::remove_if(self->block_frontiers_.begin(), self->block_frontiers_.end(),
                               [&](const Vec2& q){ return dist2(p, q) < self->block_merge_eps_; }),
                self->block_frontiers_.end());
        }

        bool isBlocked(const Vec2& f) const {
            for (const auto& bf : self->block_frontiers_) {
                if (dist2(f, bf) < self->block_merge_eps_) return true;
            }
            return false;
        }

        // Merge + filter + CLUSTER de-dup frontiers for this round
        void buildMergedFrontiers() {
            std::vector<Vec2> raw;
            raw.reserve(2048);

            for (int i = 0; i < self->num_robot_; ++i) {
                for (const auto& f : snap.frontiers_by_robot[i]) {
                    raw.push_back(f);
                }
            }

            // Filter blocked (soft + perm)
            if (!raw.empty() && (!self->block_frontiers_.empty() || !self->perm_block_frontiers_.empty())) {
                std::vector<Vec2> filtered;
                filtered.reserve(raw.size());
                for (const auto& f : raw) {
                    if (isBlocked(f)) continue;
                    if (isPermBlocked(f)) continue;
                    filtered.push_back(f);
                }
                raw.swap(filtered);
            }

            // If no frontiers, re-inject SOFT blocked list as candidates (optional)
            // 注意：永久封禁绝对不能 reinject
            if (raw.empty() && !self->block_frontiers_.empty()) {
                raw = self->block_frontiers_;
                // reinject 后也要把永久封禁再过滤一遍
                if (!self->perm_block_frontiers_.empty()) {
                    std::vector<Vec2> filtered;
                    filtered.reserve(raw.size());
                    for (const auto& f : raw) if (!isPermBlocked(f)) filtered.push_back(f);
                    raw.swap(filtered);
                }
            }

            // If no frontiers, re-inject blocked list as candidates
            if (raw.empty() && !self->block_frontiers_.empty()) {
                raw = self->block_frontiers_;
            }

            // Cluster de-dup -> frontiers (representatives)
            frontiers.clear();
            frontiers.reserve(raw.size());

            for (const auto& p : raw) {
                int best = -1;
                double best_d = std::numeric_limits<double>::infinity();
                for (int i = 0; i < (int)frontiers.size(); ++i) {
                    const double d = dist2(p, frontiers[i]);
                    if (d < best_d) { best_d = d; best = i; }
                }
                if (best != -1 && best_d < self->frontier_merge_eps_) {
                    // keep representative as-is (stable)
                    continue;
                } else {
                    frontiers.push_back(p);
                }
            }
        }

        bool freezeIfNoFrontiers() {
            const bool no_frontiers = frontiers.empty();
            self->no_frontiers_ = no_frontiers;

            if (no_frontiers) {
                ROS_WARN_THROTTLE(1.0, "No Frontiers Available!");
                self->no_frontiers_ct++;
                if (self->no_frontiers_ct > 50) {
                    ROS_INFO("\033[40;36m No Frontiers for a long time! Navigation Done! \033[0m");
                    out.navigation_done = true;
                    return true;
                }

                // Output = last
                std::vector<bool> is_temp_freeze(self->num_robot_, true);
                if (snap.last_is_temp.size() == (size_t)self->num_robot_) {
                    is_temp_freeze = snap.last_is_temp;
                }

                for (int r = 0; r < self->num_robot_; ++r) {
                    if (snap.has_target.size() == (size_t)self->num_robot_ &&
                        snap.last_targets.size() == (size_t)self->num_robot_ &&
                        snap.has_target[r] && isFiniteVec2(snap.last_targets[r])) {
                        out.targets_xy[r] = snap.last_targets[r];
                    } else {
                        out.targets_xy[r] = poseXY(r);
                        is_temp_freeze[r] = true;
                    }
                }

                for (int r = 0; r < self->num_robot_; ++r) {
                    self->publishMoveBaseSimpleGoal(r, out.targets_xy[r], snap.poses[r]);
                }

                // Update memory using frozen output
                {
                    std::lock_guard<std::mutex> lock(self->mtx_);
                    for (int r = 0; r < self->num_robot_; ++r) {
                        if (isFiniteVec2(out.targets_xy[r])) {
                            self->last_targets_xy_[r] = out.targets_xy[r];
                            self->last_is_temp_[r]    = is_temp_freeze[r];
                            self->has_target_[r]      = true;

                            // DP-only green memory: update only if frozen identity is GREEN
                            if (!is_temp_freeze[r]) {
                                self->last_green_targets_xy_[r] = out.targets_xy[r];
                                self->has_green_target_[r]      = true;
                            }
                        }
                    }
                }
                return true;
            } else {
                self->no_frontiers_ct = 0;
            }

            return false;
        }

        Vec2 computeRefPoint() {
            Vec2 ref_point{0.0, 0.0};
            int ref_cnt = 0;

            if (snap.has_target.size() == (size_t)self->num_robot_ &&
                snap.last_is_temp.size() == (size_t)self->num_robot_) {
                for (int r = 0; r < self->num_robot_; ++r) {
                    if (!snap.has_target[r]) continue;
                    if (snap.last_is_temp[r]) continue; // only last GREEN
                    const Vec2& t = snap.last_targets[r];
                    if (!isFiniteVec2(t)) continue;
                    ref_point.first  += t.first;
                    ref_point.second += t.second;
                    ref_cnt++;
                }
            }

            if (ref_cnt > 0) {
                ref_point.first  /= (double)ref_cnt;
                ref_point.second /= (double)ref_cnt;
            } else {
                for (int r = 0; r < self->num_robot_; ++r) {
                    ref_point.first  += snap.poses[r].position.x;
                    ref_point.second += snap.poses[r].position.y;
                }
                ref_point.first  /= (double)self->num_robot_;
                ref_point.second /= (double)self->num_robot_;
            }

            return ref_point;
        }

        Vec2 applyRefInertia(const Vec2& ref_point) {
            Vec2 ref_use = ref_point;
            if (self->has_last_ref_point_) {
                Vec2 dir{ref_point.first  - self->last_ref_point_.first,
                         ref_point.second - self->last_ref_point_.second};
                const double dir_norm = std::hypot(dir.first, dir.second);

                if (std::isfinite(dir_norm) && dir_norm > self->ref_inertia_min_step_) {
                    double step = self->ref_inertia_alpha_ * dir_norm;
                    step = std::min(step, self->ref_inertia_max_step_);
                    Vec2 dir_unit{dir.first / dir_norm, dir.second / dir_norm};
                    ref_use.first  = ref_point.first  + step * dir_unit.first;
                    ref_use.second = ref_point.second + step * dir_unit.second;
                }
            }
            return ref_use;
        }

        struct Item {
            double dist_target_frontier;
            double dist_robot_frontier;
            int r;
            int k; // fid for unique frontier
        };

        std::vector<Item> buildSortedFrontierItems(const Vec2& target_frontier) const {
            std::vector<Item> items;
            items.reserve((size_t)self->num_robot_ * frontiers.size());
            for (int r = 0; r < self->num_robot_; ++r) {
                for (int k = 0; k < (int)frontiers.size(); ++k) {
                    const Vec2& f = frontiers[k];
                    const double dist_rf = std::hypot(snap.poses[r].position.x - f.first,
                                                      snap.poses[r].position.y - f.second);
                    const double dist_tf = dist2(target_frontier, f);
                    items.push_back({dist_tf, dist_rf, r, k});
                }
            }

            std::sort(items.begin(), items.end(),
                      [](const Item& a, const Item& b) {
                          if (a.dist_target_frontier != b.dist_target_frontier)
                              return a.dist_target_frontier < b.dist_target_frontier;
                          if (a.dist_robot_frontier != b.dist_robot_frontier)
                              return a.dist_robot_frontier < b.dist_robot_frontier;
                          if (a.r != b.r) return a.r < b.r;
                          return a.k < b.k;
                      });
            return items;
        }

        bool solveMinAssignmentDP(const std::vector<int>& robots,
                                  const std::vector<Vec2>& targets,
                                  std::vector<int>& out_match) const
        {
            const int n = (int)robots.size();
            if (n == 0) return true;
            if ((int)targets.size() != n) return false;

            const int FULL = (1 << n);
            std::vector<double> dp(FULL, std::numeric_limits<double>::infinity());
            std::vector<int> parent_mask(FULL, -1);
            std::vector<int> parent_choice(FULL, -1);

            dp[0] = 0.0;

            for (int mask = 0; mask < FULL; ++mask) {
                const int i = __builtin_popcount((unsigned)mask);
                if (i >= n) continue;
                const int r = robots[i];
                for (int j = 0; j < n; ++j) {
                    if (mask & (1 << j)) continue;

                    const Vec2& t = targets[j];
                    double cost = dist2(poseXY(r), t);

                    const int nmask = mask | (1 << j);
                    if (dp[mask] + cost < dp[nmask]) {
                        dp[nmask] = dp[mask] + cost;
                        parent_mask[nmask] = mask;
                        parent_choice[nmask] = j;
                    }
                }
            }

            int mask = FULL - 1;
            if (!std::isfinite(dp[mask])) return false;

            out_match.assign(n, -1);
            for (int i = n - 1; i >= 0; --i) {
                const int j = parent_choice[mask];
                if (j < 0) return false;
                out_match[i] = j;
                mask = parent_mask[mask];
            }
            return true;
        }

        void assignGreens(const std::vector<Item>& dist_items) {
            // 0) PRE-FILL: keep last GREEN target for robots that haven't reached it yet.
            // This prevents "GREEN -> ORANGE" just because DP did not include the robot.
            for (int r = 0; r < self->num_robot_; ++r) {
                if (snap.has_green_target.size() != (size_t)self->num_robot_) break;
                if (!snap.has_green_target[r]) continue;
                
                const Vec2 lastg = snap.last_green_targets[r];
                if (isPermBlocked(lastg)) continue;
                if (!isFiniteVec2(lastg)) continue;

                const double d = dist2(poseXY(r), lastg);

                // not reached -> keep as GREEN
                if (std::isfinite(d) && d > self->reach_radius_) {
                    // Optional: if you want strict feasibility check, uncomment:
                    if (!canAssignGreen(r, lastg)) continue;

                    out.targets_xy[r] = lastg;
                    is_temp[r] = false;
                    assigned_robots.insert(r);
                    green_set.push_back(r);
                    robot_assigned_fid[r] = -1;  // not a frontier resource
                }
            }
            // 1) DP pool from LAST GREEN ONLY (anti-pollution)
            std::vector<int> eligible_robots;
            std::vector<Vec2> targets_pool;
            eligible_robots.reserve(self->num_robot_);
            targets_pool.reserve(self->num_robot_);

            for (int r = 0; r < self->num_robot_; ++r) {
                if (assigned_robots.count(r)) continue;  
                if (snap.has_green_target.size() != (size_t)self->num_robot_) break;
                if (!snap.has_green_target[r]) continue;

                const Vec2 cand = snap.last_green_targets[r];
                if (!isFiniteVec2(cand)) continue;

                const double d = dist2(poseXY(r), cand);
                if (d <= self->reach_radius_) continue;

                eligible_robots.push_back(r);
                targets_pool.push_back(cand);
            }

            // 2) Solve min-cost assignment
            std::vector<int> match;
            const bool ok = solveMinAssignmentDP(eligible_robots, targets_pool, match);

            // 3) Apply with greedy repair under constraints
            if (ok) {
                std::vector<int> order((int)eligible_robots.size());
                for (int i = 0; i < (int)order.size(); ++i) order[i] = i;
                std::sort(order.begin(), order.end(),
                          [&](int a, int b){ return eligible_robots[a] < eligible_robots[b]; });

                for (int idx : order) {
                    const int r = eligible_robots[idx];

                    std::vector<int> cand_t;
                    cand_t.reserve(targets_pool.size());
                    if (match[idx] >= 0) cand_t.push_back(match[idx]);

                    std::vector<std::pair<double,int>> others;
                    others.reserve(targets_pool.size());
                    for (int j = 0; j < (int)targets_pool.size(); ++j) {
                        if (j == match[idx]) continue;
                        others.push_back({dist2(poseXY(r), targets_pool[j]), j});
                    }
                    std::sort(others.begin(), others.end());
                    for (auto& pr : others) cand_t.push_back(pr.second);

                    for (int j : cand_t) {
                        const Vec2 cand = targets_pool[j];

                        // additional de-dup (avoid near-equal)
                        bool used = false;
                        for (int g : green_set) {
                            if (!is_temp[g] && isFiniteVec2(out.targets_xy[g]) &&
                                dist2(out.targets_xy[g], cand) < self->goal_dedup_eps_) {
                                used = true; break;
                            }
                        }
                        if (used) continue;

                        if (!canAssignGreen(r, cand)) continue;

                        out.targets_xy[r] = cand;
                        is_temp[r] = false;
                        eraseBlockedIfNear(cand);
                        assigned_robots.insert(r);
                        green_set.push_back(r);

                        // DP target is not "current frontier resource"
                        robot_assigned_fid[r] = -1;
                        break;
                    }
                }
            }

            // 4) Fill remaining robots with UNIQUE frontier candidates (greedy)
            for (const auto& it : dist_items) {
                const int r = it.r;
                const int fid = it.k;

                if (assigned_robots.count(r)) continue;
                if (assigned_fids.count(fid)) continue;

                const Vec2 cand = frontiers[fid];
                if (!canAssignGreen(r, cand)) continue;

                out.targets_xy[r] = cand;
                is_temp[r] = false;
                eraseBlockedIfNear(cand);

                assigned_robots.insert(r);
                assigned_fids.insert(fid);
                green_set.push_back(r);
                robot_assigned_fid[r] = fid;

                if ((int)assigned_robots.size() == self->num_robot_) break;
            }
        }

        void globalPairwiseFix(int max_it, const char* tag) {
            auto violatesPair = [&](int a, int b)->bool {
                if (a == b) return false;
                if (is_temp[a] || is_temp[b]) return false;
                if (!isFiniteVec2(out.targets_xy[a]) || !isFiniteVec2(out.targets_xy[b])) return false;

                if (!directionOK(a, out.targets_xy[a], b, out.targets_xy[b])) return true;
                if (!directionOK(b, out.targets_xy[b], a, out.targets_xy[a])) return true;

                const double d = dist2(out.targets_xy[a], out.targets_xy[b]);
                if (d < self->MIN_TGT_DIST || d > self->MAX_TGT_DIST) return true;

                if (!self->isLineFreeOnMap(out.targets_xy[a], out.targets_xy[b])) return true;

                // also de-dup check (extra safety)
                if (d < self->goal_dedup_eps_) return true;

                return false;
            };

            auto dropToOrange = [&](int r) {
                // 1) 关键：标记本轮被drop
                dropped_green[r] = true;

                // 2) 关键：强制发布，避免你shouldPublish门控导致“状态没同步”
                force_publish[r] = true;

                // release occupied frontier fid if any
                if (robot_assigned_fid[r] != -1) {
                    assigned_fids.erase(robot_assigned_fid[r]);
                    robot_assigned_fid[r] = -1;
                }

                out.targets_xy[r] = NaN2();
                is_temp[r] = true;
                green_set.erase(std::remove(green_set.begin(), green_set.end(), r), green_set.end());
                assigned_robots.erase(r);
            };

            for (int it = 0; it < max_it; ++it) {
                bool changed = false;
                for (size_t i = 0; i < green_set.size(); ++i) {
                    for (size_t j = i + 1; j < green_set.size(); ++j) {
                        const int a = green_set[i];
                        const int b = green_set[j];
                        if (!violatesPair(a, b)) continue;

                        const double da = dist2(poseXY(a), out.targets_xy[a]);
                        const double db = dist2(poseXY(b), out.targets_xy[b]);
                        const int drop = (da >= db) ? a : b;

                        ROS_WARN("\033[41;37m %s drop r%d GREEN->ORANGE due to pairwise conflict\033[0m",
                                 tag, drop + 1);
                        dropToOrange(drop);

                        changed = true;
                        break;
                    }
                    if (changed) break;
                }
                if (!changed) break;
            }
        }

        std::vector<bool> forceStopGreensByLoS(double th) {
            std::vector<bool> forced(self->num_robot_, false);
            (void)th;
            return forced;
        }

        void assignOrangeTemps(const std::vector<bool>& forced_stop_green) {
            (void)forced_stop_green;

            auto computeCellCost = [&](int ego, int ref, const Vec2& cell)->double {
                const int ego_side  = frontSignWrtRef(ref, poseXY(ego));
                const int cell_side = frontSignWrtRef(ref, cell);
                if (ego_side != 0 && cell_side != 0 && ego_side != cell_side) return 1e5;

                const double yaw_ref  = yawFromTo(ego, poseXY(ref));
                const double yaw_this = yawFromTo(ego, cell);
                const double delta = shortestAbsYawDiff(yaw_ref, yaw_this);
                if (delta > M_PI / 2) return 1e5;

                return hypot(cell.first - snap.poses[ego].position.x,
                             cell.second - snap.poses[ego].position.y);
            };

            auto evaluateNeighboringTarget = [&](int ego, int ref, Vec2& out_tmp)->bool {
                if (std::hypot(snap.poses[ego].position.x - snap.poses[ref].position.x,
                               snap.poses[ego].position.y - snap.poses[ref].position.y) <= 1.8) {
                    out_tmp = poseXY(ego);
                    return true;
                }

                if (snap.neighbor_pc.size() != (size_t)self->num_robot_) return false;
                const auto& pc = snap.neighbor_pc[ref];
                if (pc.empty()) return false;

                auto tooCloseToExistingTargets = [&](int ego_idx, const Vec2& p)->bool {
                    for (int i = 0; i < (int)out.targets_xy.size(); ++i) {
                        if (i == ego_idx) continue;
                        if (!isFiniteVec2(out.targets_xy[i])) continue;
                        // de-dup uses goal_dedup_eps_ (NOT 1.8)
                        if (dist2(out.targets_xy[i], p) < self->goal_dedup_eps_) return true;
                    }
                    return false;
                };

                double best = std::numeric_limits<double>::infinity();
                Vec2 best_pt = poseXY(ego);
                bool picked = false;

                for (const auto& q : pc) {
                    Vec2 cand{(double)q.x(), (double)q.y()};
                    if (tooCloseToExistingTargets(ego, cand)) continue;
                    if (checkIRCollision(ego, cand)) continue;

                    const double c = computeCellCost(ego, ref, cand);
                    if (!std::isfinite(c)) continue;

                    if (c < best) {
                        best = c;
                        best_pt = cand;
                        picked = true;
                    }
                }

                if (!picked) return false;
                out_tmp = best_pt;
                return true;
            };

            for (int r = 0; r < self->num_robot_; ++r) {
                if (isFiniteVec2(out.targets_xy[r])) continue; // already GREEN

                Vec2 tmp = poseXY(r);
                std::vector<int> nei = (snap.neighbors.size() == (size_t)self->num_robot_) ? snap.neighbors[r]
                                                                                           : std::vector<int>{};

                while (!nei.empty()) {
                    int ref = -1;

                    // prefer GREEN neighbor first (farthest GREEN target)
                    {
                        int best = -1;
                        double best_d = -std::numeric_limits<double>::infinity();
                        for (int n : nei) {
                            if (n < 0 || n >= self->num_robot_ || n == r) continue;
                            if (!isFiniteVec2(out.targets_xy[n]) || is_temp[n]) continue; // must be GREEN
                            double d = dist2(poseXY(r), out.targets_xy[n]);
                            if (d > best_d) { best_d = d; best = n; }
                        }
                        if (best != -1) ref = best;
                    }

                    // fallback: farthest neighbor robot
                    if (ref == -1) {
                        int best = -1;
                        double best_d = -std::numeric_limits<double>::infinity();
                        for (int n : nei) {
                            if (n < 0 || n >= self->num_robot_ || n == r) continue;
                            double d = dist2(poseXY(r), poseXY(n));
                            if (d > best_d) { best_d = d; best = n; }
                        }
                        ref = best;
                    }

                    nei.erase(std::remove(nei.begin(), nei.end(), ref), nei.end());

                    if (ref < 0 || ref >= self->num_robot_ || ref == r) continue;
                    if (snap.has_pc.size() == (size_t)self->num_robot_ && !snap.has_pc[ref]) continue;

                    if (evaluateNeighboringTarget(r, ref, tmp)) break;
                }

                out.targets_xy[r] = tmp;
                is_temp[r] = true;
                robot_assigned_fid[r] = -1;
            }
        }

        void publishMarkers() {
            visualization_msgs::Marker mk_assigned =
                self->makeSphereListMarker("assigned_targets", 0, self->makeColor(0.0f, 1.0f, 0.0f, 1.0f));
            visualization_msgs::Marker mk_temp =
                self->makeSphereListMarker("temp_targets", 0, self->makeColor(1.0f, 0.55f, 0.0f, 1.0f));

            for (int r = 0; r < self->num_robot_; ++r) {
                geometry_msgs::Point p;
                p.x = out.targets_xy[r].first;
                p.y = out.targets_xy[r].second;
                p.z = 0.05;
                if (is_temp[r]) mk_temp.points.push_back(p);
                else            mk_assigned.points.push_back(p);
            }

            self->pub_assigned_targets_marker_.publish(mk_assigned);
            self->pub_temp_targets_marker_.publish(mk_temp);
        }

        void publishTargetType() {
            std_msgs::Bool msg;
            for (int r = 0; r < self->num_robot_; ++r) {
                msg.data = (!is_temp[r]);
                self->pub_targets_type_[r].publish(msg);
            }
        }

        void printTargets() const {
            std::ostringstream oss;
            oss << "Assigned targets:";
            for (int r = 0; r < self->num_robot_; ++r) {
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

        void publishGoalsAndUpdateMemory() {
            auto shouldPublish = [&](int r)->bool {
                const Vec2& newg = out.targets_xy[r];
                if (!isFiniteVec2(newg)) return false;

                if (!self->has_published_goal_[r]) return true;
                if (self->last_published_is_temp_[r] != is_temp[r]) return true;

                const double d = dist2(newg, self->last_published_goal_[r]);
                return d > 0.20;
            };

            std::vector<bool> published_this_round(self->num_robot_, false);

            for (int r = 0; r < self->num_robot_; ++r) {
                if (force_publish[r] || shouldPublish(r)) {
                    self->publishMoveBaseSimpleGoal(r, out.targets_xy[r], snap.poses[r]);
                    self->last_published_goal_[r] = out.targets_xy[r];
                    self->has_published_goal_[r] = true;
                    self->last_published_is_temp_[r] = is_temp[r];
                    published_this_round[r] = true;
                }
            }

            // Update memory:
            // 1) last_targets_xy_ can record ORANGE (for display/jump-guard/freeze)
            // 2) last_green_targets_xy_ ONLY updates on GREEN (anti-DP-pollution)
            {
                std::lock_guard<std::mutex> lock(self->mtx_);
                for (int r = 0; r < self->num_robot_; ++r) {
                    // ===== 新增：如果本轮被drop，清掉GREEN记忆，防止下轮Step0复活 =====
                    if (dropped_green[r]) {
                        self->has_green_target_[r] = false;
                        // 可选：也清掉坐标，调试更直观
                        self->last_green_targets_xy_[r] = NaN2();
                    }
                    if (!published_this_round[r]) continue;
                    if (!isFiniteVec2(out.targets_xy[r])) continue;

                    self->last_targets_xy_[r] = out.targets_xy[r];
                    self->last_is_temp_[r]    = is_temp[r];
                    self->has_target_[r]      = true;

                    if (!is_temp[r]) {
                        self->last_green_targets_xy_[r] = out.targets_xy[r];
                        self->has_green_target_[r]      = true;
                    }
                }
            }
        }
    };

    // -------------------------
    // Main pipeline
    // -------------------------
    Ctx ctx(this, snap, out);
    // Step 0: handle emergency stop -> permanently ban its last target and clear its memory
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for (int r = 0; r < num_robot_; ++r) {
            if (snap.emerg_stop.size() != (size_t)num_robot_) break;
            if (!snap.emerg_stop[r]) continue;

            // 1) 用“上一次发布/记忆的目标”作为要封禁的点（最合理）
            Vec2 bad = {std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN()};

            if (has_target_[r] && isFiniteVec2(last_targets_xy_[r])) {
                bad = last_targets_xy_[r];
            } else if (isFiniteVec2(snap.last_targets[r])) {
                bad = snap.last_targets[r];
            }

            // 永久封禁
            if (isFiniteVec2(bad)) {
                // 这里不能直接 ctx.addPermBlocked(bad)，因为 ctx 内部是引用 self->perm_block_frontiers_
                // 但你现在在 self 方法里，直接操作 self 的成员也行：
                bool existed = false;
                for (const auto& bf : perm_block_frontiers_) {
                    if (std::hypot(bad.first - bf.first, bad.second - bf.second) < perm_block_merge_eps_) {
                        existed = true; break;
                    }
                }
                if (!existed) perm_block_frontiers_.push_back(bad);

                ROS_WARN("\033[41;37m EmergencyStop r%d: permanently block target (%.2f, %.2f)\033[0m",
                        r + 1, bad.first, bad.second);
            }

            // 2) 清掉该机器人的目标记忆，避免 Step0 复活
            has_target_[r] = false;
            last_targets_xy_[r] = {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN()};
            last_is_temp_[r] = true;

            has_green_target_[r] = false;
            last_green_targets_xy_[r] = {std::numeric_limits<double>::quiet_NaN(),
                                        std::numeric_limits<double>::quiet_NaN()};

            // 3) 可选：强制下一次发布一个“原地 ORANGE”目标，避免 move_base 继续朝坏点走
            // 你在后面 ORANGE 分配会给它 tmp=poseXY(r)，这里也可以不做
        }
    }
    // Step A: merge + de-dup frontiers
    ctx.buildMergedFrontiers();

    // Step B: freeze if no frontiers
    if (ctx.freezeIfNoFrontiers()) {
        return out;
    }

    // Step C: pick inertia frontier
    const Vec2 ref_point = ctx.computeRefPoint();
    const Vec2 ref_use   = ctx.applyRefInertia(ref_point);

    int closest_k = 0;
    double best = std::numeric_limits<double>::infinity();
    for (int k = 0; k < (int)ctx.frontiers.size(); ++k) {
        const double d = ctx.dist2(ref_use, ctx.frontiers[k]);
        if (d < best) { best = d; closest_k = k; }
    }
    const Vec2 target_frontier = ctx.frontiers[closest_k];

    last_ref_point_ = ref_point;
    has_last_ref_point_ = true;

    const auto dist_items = ctx.buildSortedFrontierItems(target_frontier);

    // Step D: assign GREEN (DP on clean last_green_targets + fill by unique frontiers)
    ctx.assignGreens(dist_items);

    // Step E: global pairwise fix (also releases frontier fid on drop)
    ctx.globalPairwiseFix(4, "GlobalFix");

    // Step H: force-stop greens (currently disabled)
    const auto forced_stop_green = ctx.forceStopGreensByLoS(-1);

    // Step I: assign ORANGE
    ctx.assignOrangeTemps(forced_stop_green);

    // Step K: visualization
    ctx.publishMarkers();
    ctx.publishTargetType();
    ctx.printTargets();

    // Step L: publish goals + update memory (with gating + DP-only green memory)
    ctx.publishGoalsAndUpdateMemory();

    return out;
}

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_assigner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    FrontierAssigner assigner(nh, pnh);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (!assigner.allPosesReady() || !assigner.allConvexHullsReady() 
        ||
            !assigner.allFrontiersReady() || !assigner.GraphReceived() ||
            !assigner.allPCReady() || !assigner.MapReceived()
        ) {
            rate.sleep();
            continue;
        }
        // star_convex_handle::StarConvexHandler scp(nh);
        // auto los12 = scp.starConvexConstraint({assigner.get_poses()[0].position.x, assigner.get_poses()[0].position.y}, {assigner.get_poses()[1].position.x, assigner.get_poses()[1].position.y}, assigner.get_convex_hulls()[1], false);
        // auto los13 = scp.starConvexConstraint({assigner.get_poses()[0].position.x, assigner.get_poses()[0].position.y}, {assigner.get_poses()[2].position.x, assigner.get_poses()[2].position.y}, assigner.get_convex_hulls()[2], false);
        // ROS_INFO("LOS12: %f, LOS13: %f", los12(0), los13(0));
        auto res = assigner.assignCloseFrontiers();
        if (res.navigation_done) break;

        rate.sleep();
    }

    return 0;
}