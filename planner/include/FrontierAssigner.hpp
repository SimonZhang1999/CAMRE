#pragma once

// ============================================================
// ROS / Messages
// ============================================================
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <cure_planner/PointArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// TF
#include "tf/tf.h"
#include "tf/transform_listener.h"

// Eigen
#include <Eigen/Dense>

// ============================================================
// STL
// ============================================================
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Project
#include "star_convex_handle.h"

class FrontierAssigner {
public:
    using Vec2 = std::pair<double, double>;
    using TimerStayMap = std::unordered_map<int, int>; // robot_id -> remaining stay ticks

    struct AssignResult {
        int leader_idx = -1;            // -1 if none
        std::vector<Vec2> targets_xy;   // size = num_robot, NaN if none
        bool navigation_done = false;
    };

    struct Snapshot {
        std::vector<geometry_msgs::Pose> poses;
        std::vector<std::vector<Vec2>> frontiers_by_robot;
        std::vector<Vec2> merged_frontiers;

        std::vector<std::vector<int>> neighbors;
        std::vector<std::vector<Eigen::Vector2d>> hulls;
        std::vector<std::vector<Eigen::Vector2f>> neighbor_pc;

        std::vector<bool> has_target;
        std::vector<bool> last_is_temp;
        std::vector<Vec2> last_targets;

        std::vector<bool> emerg_stop;

        std::vector<Vec2> last_green_targets;
        std::vector<bool> has_green_target;

        std::vector<bool> has_pose, has_hull, has_pc, has_frontier;
        bool have_graph = false;
    };

public:
    explicit FrontierAssigner(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // =========================
    // Main API
    // =========================
    AssignResult assignCloseFrontiers();
    std::vector<Vec2> getMergedFrontiers() const;

    // =========================
    // Callbacks
    // =========================
    void poseCb(const nav_msgs::Odometry::ConstPtr& msg, int robot_idx);
    void frontiersCb(const cure_planner::PointArrayConstPtr& msg, int robot_idx);
    void convexhullsCb(const std_msgs::Float32MultiArray::ConstPtr& msg, int idx);
    void edgesCb(const std_msgs::Float32MultiArrayConstPtr& msg);
    void emergencystopCb(const std_msgs::Bool::ConstPtr& msg, int robot_idx);
    void neiPcCb(const sensor_msgs::PointCloud2ConstPtr& msg, int robot_idx);
    void planningMapCb(const nav_msgs::OccupancyGridConstPtr& msg);

    // =========================
    // Ready checks
    // =========================
    bool allEmergencyStopReady() const;
    bool allFrontiersReady() const;
    bool allPosesReady() const;
    bool allConvexHullsReady() const;
    bool allPCReady() const;

    inline bool GraphReceived() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return have_graph_;
    }

    inline bool MapReceived() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return has_map_;
    }

    // =========================
    // Snapshot
    // =========================
    Snapshot getSnapshotLocked() const;

    // =========================
    // Legacy helpers (kept)
    // =========================
    double distanceToNearestObstacle(const nav_msgs::OccupancyGrid& map,
                                     double wx, double wy,
                                     double search_radius_m) const;

    bool EvaluateNeiboringTarget(int& ego_idx, int& r_idx, Vec2& temp,
                                 const std::vector<Vec2>& target_set,
                                 const bool& emergency);

    double computeCellCost(int& ego_idx, int& ri_idx,
                           const Vec2& candidate_cell,
                           const bool& emergency) const;

    double computeLoSCost(int& ego_idx, int& ri_idx) const;

    bool CheckExistingTargets(const int& ego_idx, const double& x, const double& y,
                              const std::vector<Vec2>& target_set);

    bool CheckIRCollision(const int& ego_idx, const double& x, const double& y);
    bool CheckFrontierFeasibility(const std::vector<Vec2>& frontiers, const Vec2& candidate);

    bool ReachedTarget(int r, const Vec2& target, double radius) const;

    int selectNeighborRefRobot(int r,
                              const std::vector<int>& nei_idx_set,
                              const std::vector<Vec2>& assigned_targets_xy,
                              const std::vector<bool>& is_temp) const;

    bool FrontierDirectionCheck(int r_idx, const Vec2& candidate,
                               int g_idx, const Vec2& g_target) const;

    // =========================
    // Core helpers
    // =========================
    bool FrontiersFeasibility(const Vec2& frontier) const;
    std::vector<Vec2> mergeAllFrontiersLocked() const;

    void publishMoveBaseSimpleGoal(int robot_idx, const Vec2& target_xy,
                                   const geometry_msgs::Pose& robot_xy);

    static double yawFromTo(const geometry_msgs::Pose& from, const Vec2& to);
    static void yawToQuaternion(double yaw, double& qx, double& qy, double& qz, double& qw);

    // debug getters (optional)
    std::vector<std::vector<Eigen::Vector2d>> get_convex_hulls() const { return robots_convexhull_; }
    std::vector<geometry_msgs::Pose> get_poses() const { return poses_; }

    // =========================
    // Public params / states (existing)
    // =========================
    bool no_frontiers_{false}, last_no_frontiers_{false};
    int  no_frontiers_ct{0};

    double frontier_robot_min_dist_{1.5};
    double cluster_window_{0.0}, cluster_radius_{0.0};
    double inter_robot_threshold_{0.0};

    double frontier_merge_eps_{0.35};
    double goal_dedup_eps_{0.35};

    int all_orange_ct_{0};
    int all_orange_relax_M_{5};

    std::vector<Vec2> stuck_retry_frontiers_;
    double stuck_retry_merge_eps_{0.35};

    std::vector<Vec2> perm_block_frontiers_;
    double perm_block_merge_eps_{0.30};

    std::vector<Vec2> last_published_goal_;
    std::vector<bool> has_published_goal_;
    std::vector<bool> last_published_is_temp_;
    int should_publish_count_{0};

    std::vector<Vec2> last_green_targets_xy_;
    std::vector<bool> has_green_target_;

    std::vector<int> jump_guard_ct_;

    // retry (sticky takeover)
    bool retry_active_{false};
    int  retry_robot_{-1};
    Vec2 retry_target_{std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN()};

    double idle_clearance_search_radius_{6.0};

    // jump guard thresholds
    double jump_guard_dist_th_{20.0};
    double jump_guard_yaw_th_{90.0};
    int    jump_guard_max_ct_{3};

    // stuck detection
    std::vector<Vec2> last_pose_xy_;
    std::vector<double> last_dist_to_goal_;
    std::vector<int> stuck_ct_;
    double stuck_move_eps_{0.1};
    double stuck_prog_eps_{0.1};
    int    stuck_cycles_{25};

    // blocked frontiers
    std::vector<Vec2> block_frontiers_;
    double block_merge_eps_{0.30};

    // forbidden targets per robot
    std::vector<std::vector<Vec2>> robot_forbidden_targets_;
    double robot_forbidden_merge_eps_{0.35};

    // anchor pause/jump state
    int anchor_pause_ct_{0};
    int anchor_pause_K_{3};

    Vec2 last_round_green_ref_{std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::quiet_NaN()};
    bool has_last_round_green_ref_{false};

    std::pair<double,double> last_round_green_dir_{0.0, 0.0};
    bool has_last_round_green_dir_{false};

    bool allow_anchor_jump_{false};

    // anchor blocked-LOS lock
    bool anchor_lock_active_{false};
    int  anchor_lock_robot_{-1};
    Vec2 anchor_lock_target_{std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN()};
    double anchor_lock_match_eps_{0.6};

    // anchor history (reached green targets)
    std::vector<Vec2> anchor_green_history_;
    double anchor_hist_drop_eps_{2.0};
    double anchor_hist_merge_eps_{0.30};
    int    anchor_hist_max_size_{40000};

    double los_penalty_{1000.0};

    void rememberAnchorTarget(const Vec2& p);
    bool nearAnyAnchorHistory(const Vec2& p) const;
    void pruneAnchorHistoryIfNeeded();

private:
    // ============================================================
    // ROS
    // ============================================================
    ros::NodeHandle nh_;
    std::shared_ptr<star_convex_handle::StarConvexHandler> scp_;

    ros::Subscriber edges_sub_;
    ros::Subscriber sub_planning_map_;
    std::vector<ros::Subscriber> pose_subs_;
    std::vector<ros::Subscriber> frontier_subs_;
    std::vector<ros::Subscriber> subs_vertices_convexhull_;
    std::vector<ros::Subscriber> emergency_stop_subs_;
    std::vector<ros::Subscriber> PC_subs_;

    std::vector<ros::Publisher> goal_pubs_;
    ros::Publisher pub_assigned_targets_marker_;
    ros::Publisher pub_temp_targets_marker_;
    ros::Publisher pub_clustered_frontiers_marker_;
    ros::Publisher pub_record_traj_info_;
    std::vector<ros::Publisher> pub_targets_type_;

    // ============================================================
    // Shared states (mtx_)
    // ============================================================
    mutable std::mutex mtx_;
    int num_robot_{0};

    std::vector<geometry_msgs::Pose> poses_;
    std::vector<std::vector<double>> record_traj_info_;
    std::vector<std::vector<int>> robot_neighbors_;
    std::vector<std::vector<Vec2>> frontiers_by_robot_;

    std::vector<std::vector<Eigen::Vector2d>> robots_convexhull_;
    std::vector<std::vector<Eigen::Vector2f>> robots_neighbor_pc_;

    std::vector<bool> has_pose_;
    std::vector<bool> has_convexhulls_;
    std::vector<bool> has_frontiers_;
    std::vector<bool> has_emergencystop_;
    std::vector<bool> has_pc_;

    std::vector<bool> emergencystop_robots_;

    std::vector<bool> has_target_;
    std::vector<Vec2> last_targets_xy_;
    std::vector<bool> last_is_temp_;

    bool have_graph_{false};

    // direction inertia reference
    bool has_last_ref_point_{false};
    Vec2 last_ref_point_{0.0, 0.0};

    // ============================================================
    // Map (map_mtx_)
    // ============================================================
    mutable std::mutex map_mtx_;
    nav_msgs::OccupancyGrid map_;
    bool has_map_{false};

    // ============================================================
    // Params
    // ============================================================
    std::vector<std::string> frontier_topics_;
    std::string global_frame_{"map"};

    double MIN_TGT_DIST{0.0};
    double MAX_TGT_DIST{0.0};

    double target_marker_scale_{0.35};
    double marker_lifetime_{0.3};
    double reach_radius_{0.6};
    double max_heading_angle_deg_{60.0};

    int  occ_threshold_{50};
    bool treat_unknown_as_occupied_{false};
    double los_check_step_{0.0};
    int  los_inflate_cells_{1};

    double ref_inertia_alpha_{1.0};
    double ref_inertia_max_step_{1.0};
    double ref_inertia_min_step_{0.5};

    double max_green_robot_anchor_dist_{10.0};

private:
    // ============================================================
    // Marker utils
    // ============================================================
    static std_msgs::ColorRGBA makeColor(float r, float g, float b, float a=1.0f) {
        std_msgs::ColorRGBA c;
        c.r = r; c.g = g; c.b = b; c.a = a;
        return c;
    }

    visualization_msgs::Marker makeSphereListMarker(const std::string& ns, int id,
                                                    const std_msgs::ColorRGBA& color) const {
        visualization_msgs::Marker mk;
        mk.header.frame_id = global_frame_;
        mk.header.stamp = ros::Time::now();
        mk.ns = ns;
        mk.id = id;
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = target_marker_scale_;
        mk.scale.y = target_marker_scale_;
        mk.scale.z = target_marker_scale_;
        mk.color = color;
        mk.lifetime = ros::Duration(marker_lifetime_);
        return mk;
    }

    // ============================================================
    // Basic math / geometry utils
    // ============================================================
    static inline bool isFiniteVec2(const Vec2& p) {
        return std::isfinite(p.first) && std::isfinite(p.second);
    }

    static inline double dist(const Vec2& a, const Vec2& b) {
        return std::hypot(a.first - b.first, a.second - b.second);
    }

    static inline Vec2 poseXY(const geometry_msgs::Pose& p) {
        return {p.position.x, p.position.y};
    }

    // ============================================================
    // Grid / LoS utils
    // ============================================================
    static inline bool worldToGrid(const nav_msgs::OccupancyGrid& map,
                                   double wx, double wy,
                                   int& gx, int& gy) {
        const double ox  = map.info.origin.position.x;
        const double oy  = map.info.origin.position.y;
        const double res = map.info.resolution;
        if (res <= 0.0) return false;

        gx = (int)std::floor((wx - ox) / res);
        gy = (int)std::floor((wy - oy) / res);

        if (gx < 0 || gy < 0) return false;
        if (gx >= (int)map.info.width || gy >= (int)map.info.height) return false;
        return true;
    }

    static inline int gridValueAt(const nav_msgs::OccupancyGrid& map, int gx, int gy) {
        const int k = gy * (int)map.info.width + gx;
        if (k < 0 || k >= (int)map.data.size()) return 100;
        return (int)map.data[k];
    }

    inline bool worldToMap(const nav_msgs::OccupancyGrid& m,
                           double wx, double wy,
                           int& mx, int& my) const {
        const double ox  = m.info.origin.position.x;
        const double oy  = m.info.origin.position.y;
        const double res = m.info.resolution;
        if (res <= 0.0) return false;

        mx = (int)std::floor((wx - ox) / res);
        my = (int)std::floor((wy - oy) / res);

        if (mx < 0 || my < 0) return false;
        if (mx >= (int)m.info.width || my >= (int)m.info.height) return false;
        return true;
    }

    inline int idx(const nav_msgs::OccupancyGrid& m, int mx, int my) const {
        return my * (int)m.info.width + mx;
    }

    inline bool cellBlocked(const nav_msgs::OccupancyGrid& m, int mx, int my) const {
        if (mx < 0 || my < 0 || mx >= (int)m.info.width || my >= (int)m.info.height) return true;
        const int v = m.data[idx(m, mx, my)];
        if (v < 0) return treat_unknown_as_occupied_;
        return v >= occ_threshold_;
    }

    inline bool cellBlockedInflated(const nav_msgs::OccupancyGrid& m, int mx, int my) const {
        const int R = std::max(0, los_inflate_cells_);
        for (int dy = -R; dy <= R; ++dy) {
            for (int dx = -R; dx <= R; ++dx) {
                if (cellBlocked(m, mx + dx, my + dy)) return true;
            }
        }
        return false;
    }

    bool isLineFreeOnMap(const Vec2& a, const Vec2& b) const {
        nav_msgs::OccupancyGrid m;
        {
            std::lock_guard<std::mutex> lk(map_mtx_);
            if (!has_map_) return true;
            m = map_;
        }

        int x0, y0, x1, y1;
        if (!worldToMap(m, a.first, a.second, x0, y0)) return false;
        if (!worldToMap(m, b.first, b.second, x1, y1)) return false;

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0, y = y0;
        while (true) {
            if (cellBlockedInflated(m, x, y)) return false;
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return true;
    }

    // ============================================================
    // Forbidden / retry helpers
    // ============================================================
    inline bool isRobotForbidden(int r, const Vec2& p) const {
        if (r < 0 || r >= num_robot_) return false;
        if (!isFiniteVec2(p)) return false;
        for (const auto& q : robot_forbidden_targets_[r]) {
            if (dist(p, q) < robot_forbidden_merge_eps_) return true;
        }
        return false;
    }

    inline void addRobotForbidden(int r, const Vec2& p) {
        if (r < 0 || r >= num_robot_) return;
        if (!isFiniteVec2(p)) return;
        for (const auto& q : robot_forbidden_targets_[r]) {
            if (dist(p, q) < robot_forbidden_merge_eps_) return;
        }
        robot_forbidden_targets_[r].push_back(p);
    }

    inline void eraseForbiddenNearAllRobots(const Vec2& p, double eps) {
        if (!isFiniteVec2(p)) return;
        for (int r = 0; r < num_robot_; ++r) {
            auto& vec = robot_forbidden_targets_[r];
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                                     [&](const Vec2& q){
                                         return isFiniteVec2(q) && dist(p, q) < eps;
                                     }),
                      vec.end());
        }
    }

    inline void eraseForbiddenNearRobot(int r, const Vec2& p, double eps) {
        if (r < 0 || r >= (int)robot_forbidden_targets_.size()) return;
        auto& v = robot_forbidden_targets_[r];
        v.erase(std::remove_if(v.begin(), v.end(),
                               [&](const Vec2& x){ return dist(x, p) < eps; }),
                v.end());
    }

    inline void addRetryFrontier(const Vec2& p) {
        if (!isFiniteVec2(p)) return;
        for (const auto& q : stuck_retry_frontiers_) {
            if (isFiniteVec2(q) && dist(p, q) < stuck_retry_merge_eps_) return;
        }
        stuck_retry_frontiers_.push_back(p);
    }

    inline bool isForbiddenByAllRobots(const Vec2& p, double eps) const {
        if (!isFiniteVec2(p)) return false;
        const double eps2 = eps * eps;

        for (int r = 0; r < num_robot_; ++r) {
            bool has = false;
            for (const auto& q : robot_forbidden_targets_[r]) {
                if (!isFiniteVec2(q)) continue;
                const double dx = p.first  - q.first;
                const double dy = p.second - q.second;
                if (dx*dx + dy*dy < eps2) { has = true; break; }
            }
            if (!has) return false;
        }
        return true;
    }
};