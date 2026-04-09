// #ifndef _PLAN_MANAGER_HPP_
// #define _PLAN_MANAGER_HPP_

// #include "plan_env/sdf_map.h"
// #include "visualizer/visualizer.hpp"
// #include "front_end/jps_planner/jps_planner.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "back_end/optimizer.h"
// #include "tf/tf.h"
// #include "tf/transform_datatypes.h"
// #include "carstatemsgs/CarState.h"
// #include "carstatemsgs/Polynome.h"
// #include "std_msgs/Bool.h"
// #include "star_convex_handle.h"

// #include <thread>
// #include <mutex>
// #include <set>
// #include <cmath>
// #include <limits>
// #include <memory>
// #include <string>
// #include <vector>
// #include <sstream>
// #include <iomanip>
// #include <stdexcept>

// #include <nav_msgs/Odometry.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_listener.h>
// #include <std_msgs/Float64.h>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

// enum StateMachine{
//   INIT,
//   IDLE,
//   PLANNING,
//   REPLAN,
//   GOINGTOGOAL,
//   EMERGENCY_STOP,
// };

// class PlanManager
// {
//   private:
//     ros::NodeHandle nh_;

//     std::string name_space;
//     std::shared_ptr<SDFmap> sdfmap_;
//     std::shared_ptr<Visualizer> visualizer_;
//     std::shared_ptr<MSPlanner> msplanner_;
//     std::shared_ptr<JPS::JPSPlanner> jps_planner_;

//     ros::Subscriber goal_sub_;
//     ros::Subscriber current_state_sub_, current_vel_sub_;
//     ros::Subscriber sub_target_temp_, sub_graph_emergency_;
//     ros::Subscriber edges_sub_;
//     ros::Subscriber sub_weights_;
//     ros::Subscriber current_state_pose_sub_;
//     ros::Timer main_thread_timer_;
//     ros::Publisher cmd_pub_;
//     ros::Publisher mpc_polynome_pub_;
//     ros::Publisher emergency_stop_pub_;
//     ros::Publisher solving_failed_pub_;
//     ros::Publisher traj_feasibility_pub_;
//     ros::Publisher scale_down_param_pub_, graph_emergency_param_pub_;

//     ros::Publisher record_pub_;

//     ros::Time current_time_;
//     Eigen::Vector3d current_state_XYTheta_;
//     Eigen::Vector3d current_state_VAJ_;
//     Eigen::Vector3d current_state_OAJ_;

//     double plan_start_time_;
//     Eigen::Vector3d plan_start_state_XYTheta;
//     Eigen::Vector3d plan_start_state_VAJ;
//     Eigen::Vector3d plan_start_state_OAJ;

//     Eigen::Vector3d goal_state_;

//     ros::Time Traj_start_time_;
//     double Traj_total_time_;

//     ros::Time loop_start_time_;

//     bool have_geometry_ = false;
//     bool have_graph_ = false;
//     bool have_goal_ = false;
//     bool has_target_type_ = false;
//     bool target_type_ = false;
//     bool graph_emergency_ = false;

//     bool if_fix_final_ = false;
//     Eigen::Vector3d final_state_ = Eigen::Vector3d::Zero();

//     double replan_time_ = 10000.0;
//     double max_replan_time_ = 1.0;

//     double predicted_traj_start_time_ = -1.0;

//     StateMachine state_machine_ = StateMachine::INIT;

//     std::string robot_ns_;
//     int robot_index_ = 0;
//     mutable std::mutex mtx_;
//     std::vector<int> neighbors_;
//     std::vector<float> weights_;
//     int num_robots_ = 0;
//     double los_margin_, alpha_, sd_threshold_;
//     std::vector<Eigen::Vector2d> other_robots_pose_by_id_;
//     std::vector<bool> has_other_pose_by_id_;
//     std::vector<ros::Subscriber> subs_other_robots_pose_by_id_;

//     std::vector<Eigen::Vector2d> connected_robots_pose_by_id_;
//     std::vector<bool> has_connected_robots_pose_by_id_;
//     std::vector<std::vector<Eigen::Vector2d>> connected_robots_convexhull_by_id_;
//     std::vector<bool> has_connected_convexhulls_by_id_;
//     std::vector<ros::Subscriber> subs_connected_robots_pose_by_id_;
//     std::vector<ros::Subscriber> subs_connected_convexhulls_by_id_;

//     std::shared_ptr<star_convex_handle::StarConvexHandler> scp_;
//     int emerge_stop_counts = 0;
//     int stop_countdown_ = 0;
//     bool emergency_pub_state_ = false; // 可选：避免重复publish
//     int los_stop_countdown_{0};        // remaining stop rounds
//     bool los_stop_active_{false};      // last published state (optional)
//     int los_stop_rounds_{5};          // or rosparam

//   private:
//     int parseRobotIndex(const std::string& ns) {
//       // 优先从最后一个 "robot_" 后提取数字，避免 robot_ns_[7] 越界
//       std::size_t pos = ns.rfind("robot_");
//       if (pos == std::string::npos) {
//         ROS_WARN("Failed to parse robot index from namespace: %s", ns.c_str());
//         return 0;
//       }
//       pos += 6; // skip "robot_"
//       std::size_t end = pos;
//       while (end < ns.size() && std::isdigit(static_cast<unsigned char>(ns[end]))) {
//         ++end;
//       }
//       if (end == pos) {
//         ROS_WARN("Failed to parse robot index digits from namespace: %s", ns.c_str());
//         return 0;
//       }
//       try {
//         return std::stoi(ns.substr(pos, end - pos));
//       } catch (...) {
//         ROS_WARN("Exception when parsing robot index from namespace: %s", ns.c_str());
//         return 0;
//       }
//     }

//   public:
//     PlanManager(ros::NodeHandle nh){
//       nh_ = nh;
//       scp_ = std::make_shared<star_convex_handle::StarConvexHandler>(nh_);
//       nh_.param("num_robots", num_robots_, 0);
//       nh_.param("los_margin", los_margin_, 1.5);
//       nh_.param("alpha", alpha_, 0.25);
//       nh_.param("sd_threshold", sd_threshold_, 0.6);

//       other_robots_pose_by_id_.assign(num_robots_ + 1, Eigen::Vector2d::Zero());
//       has_other_pose_by_id_.assign(num_robots_ + 1, false);
//       subs_other_robots_pose_by_id_.resize(num_robots_ + 1);

//       connected_robots_pose_by_id_.assign(num_robots_ + 1, Eigen::Vector2d::Zero());
//       has_connected_robots_pose_by_id_.assign(num_robots_ + 1, false);
//       connected_robots_convexhull_by_id_.resize(num_robots_ + 1);
//       has_connected_convexhulls_by_id_.assign(num_robots_ + 1, false);
//       subs_connected_robots_pose_by_id_.resize(num_robots_ + 1);
//       subs_connected_convexhulls_by_id_.resize(num_robots_ + 1);

//       sdfmap_ = std::make_shared<SDFmap>(nh);
//       visualizer_ = std::make_shared<Visualizer>(nh);
//       msplanner_ = std::make_shared<MSPlanner>(Config(ros::NodeHandle("~")), nh_, sdfmap_);
//       jps_planner_ = std::make_shared<JPS::JPSPlanner>(sdfmap_, nh_);

//       robot_ns_ = ros::names::parentNamespace(nh_.getNamespace());
//       robot_index_ = parseRobotIndex(robot_ns_);

//       goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(robot_ns_ + "/move_base_simple/goal", 1, &PlanManager::goal_callback, this);
//       current_state_sub_ = nh_.subscribe<nav_msgs::Odometry>(robot_ns_ + "/RosAria/odom", 10, &PlanManager::GeometryCallbackPose, this);
//       sub_target_temp_ = nh_.subscribe<std_msgs::Bool>(robot_ns_ + "/target_type", 10, &PlanManager::CallbackTT, this);
//       sub_graph_emergency_ = nh_.subscribe<std_msgs::Bool>("/graph_emergency", 10, &PlanManager::CallbackGraphEmergency, this);

//       for (int id = 1; id <= num_robots_; ++id) {
//         if (id == robot_index_) continue;
//         subs_other_robots_pose_by_id_[id] = nh_.subscribe<nav_msgs::Odometry>(
//           "/robot_" + std::to_string(id) + "/RosAria/odom", 1,
//           boost::bind(&PlanManager::otherrobotsCb, this, _1, id)
//         );
//       }

//       edges_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("/mst_graph/edges", 1, &PlanManager::edgesCb, this);

//       main_thread_timer_ = nh_.createTimer(ros::Duration(0.001), &PlanManager::MainThread, this);
//       cmd_pub_ = nh_.advertise<carstatemsgs::CarState>(robot_ns_ + "/simulation/PoseSub", 1);
//       emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/emergency_stop", 1);
//       solving_failed_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/solving_failed", 1);
//       traj_feasibility_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/traj_feasibility", 1);

//       record_pub_ = nh_.advertise<visualization_msgs::Marker>(robot_ns_ + "/planner/calculator_time", 1);

//       scale_down_param_pub_ = nh_.advertise<std_msgs::Float64>(robot_ns_ + "/scale_down_pram", 1);
//       graph_emergency_param_pub_ = nh_.advertise<std_msgs::Bool>("/graph_emergency", 1);
//       mpc_polynome_pub_ = nh_.advertise<carstatemsgs::Polynome>(robot_ns_ + "/traj", 1);

//       nh_.param<bool>("if_fix_final", if_fix_final_, false);
//       if (if_fix_final_) {
//         nh_.param<double>("final_x", final_state_(0), 0.0);
//         nh_.param<double>("final_y", final_state_(1), 0.0);
//         nh_.param<double>("final_yaw", final_state_(2), 0.0);
//       }

//       nh_.param<double>("replan_time", replan_time_, 10000.0);
//       nh_.param<double>("max_replan_time", max_replan_time_, 1.0);

//       state_machine_ = StateMachine::IDLE;
//       loop_start_time_ = ros::Time::now();
//     }

//     ~PlanManager() = default;

//     void printStateMachine(){
//       if(state_machine_ == INIT) ROS_INFO("state_machine_ == INIT");
//       if(state_machine_ == IDLE) ROS_INFO("state_machine_ == IDLE");
//       if(state_machine_ == PLANNING) ROS_INFO("state_machine_ == PLANNING");
//       if(state_machine_ == REPLAN) ROS_INFO("state_machine_ == REPLAN");
//     }

//     // void GeometryCallback(const carstatemsgs::CarState::ConstPtr &msg){
//     //   have_geometry_ = true;
//     //   current_state_XYTheta_ << msg->x, msg->y, msg->yaw;
//     //   current_state_VAJ_ << msg->v, msg->a, msg->js;
//     //   current_state_OAJ_ << msg->omega, msg->alpha, msg->jyaw;
//     //   current_time_ = msg->Header.stamp;
//     // }

//     // void GeometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
//     //   have_geometry_ = true;
//     //   current_state_XYTheta_ << msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation);
//     //   current_state_VAJ_ << 0.0, 0.0, 0.0;
//     //   current_state_OAJ_ << 0.0, 0.0, 0.0;
//     //   current_time_ = msg->header.stamp;
//     // }

//     void GeometryCallbackPose(const nav_msgs::Odometry::ConstPtr &msg){
//       static tf2_ros::Buffer tf_buffer;
//       static tf2_ros::TransformListener tf_listener(tf_buffer);
//       try
//       {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp);
//         const double x = transformStamped.transform.translation.x;
//         const double y = transformStamped.transform.translation.y;
//         const double yaw = tf::getYaw(transformStamped.transform.rotation);

//         {
//           std::lock_guard<std::mutex> lock(mtx_);
//           have_geometry_ = true;
//           current_state_XYTheta_ << x, y, yaw;
//           current_state_VAJ_ << msg->twist.twist.linear.x, 0.0, 0.0;
//           current_state_OAJ_ << msg->twist.twist.angular.z, 0.0, 0.0;
//           current_time_ = ros::Time::now();
//         }
//       }
//       catch (const tf::TransformException& ex)
//       {
//         ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
//       }
//     }

//     // void VelocityCallbackPose(const nav_msgs::Odometry::ConstPtr &msg){
//     //   current_state_VAJ_ << msg->twist.twist.linear.x, 0.0, 0.0;
//     //   current_state_OAJ_ << msg->twist.twist.angular.z, 0.0, 0.0;
//     // }

//     void CallbackTT(const std_msgs::Bool::ConstPtr &msg) {
//       std::lock_guard<std::mutex> lock(mtx_);
//       has_target_type_ = true;
//       target_type_ = msg->data;
//     }

//     void CallbackGraphEmergency(const std_msgs::Bool::ConstPtr &msg) {
//       std::lock_guard<std::mutex> lock(mtx_);
//       graph_emergency_ = msg->data;
//     }

//     void otherrobotsCb(const nav_msgs::Odometry::ConstPtr &msg, int robot_id) {
//       if (robot_id < 1 || robot_id > num_robots_) return;

//       static tf2_ros::Buffer tf_buffer;
//       static tf2_ros::TransformListener tf_listener(tf_buffer);
//       try
//       {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp, ros::Duration(0.1));
//         const double x = transformStamped.transform.translation.x;
//         const double y = transformStamped.transform.translation.y;

//         std::lock_guard<std::mutex> lock(mtx_);
//         have_geometry_ = true;
//         other_robots_pose_by_id_[robot_id] = Eigen::Vector2d(x, y);
//         has_other_pose_by_id_[robot_id] = true;
//       }
//       catch (const tf::TransformException& ex)
//       {
//         ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
//       }
//     }

//     void connectedrobotsCb(const nav_msgs::Odometry::ConstPtr &msg, int robot_id) {
//       if (robot_id < 1 || robot_id > num_robots_) return;

//       static tf2_ros::Buffer tf_buffer;
//       static tf2_ros::TransformListener tf_listener(tf_buffer);
//       try
//       {
//         geometry_msgs::TransformStamped transformStamped;
//         transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp, ros::Duration(0.1));
//         const double x = transformStamped.transform.translation.x;
//         const double y = transformStamped.transform.translation.y;
//         const double yaw = tf::getYaw(transformStamped.transform.rotation);
//         (void)yaw;

//         std::lock_guard<std::mutex> lock(mtx_);
//         have_geometry_ = true;
//         connected_robots_pose_by_id_[robot_id] = Eigen::Vector2d(x, y);
//         has_connected_robots_pose_by_id_[robot_id] = true;
//       //   ROS_INFO_THROTTLE(0.2, "pose: x=%.3f y=%.3f yaw=%.3f(rad)", x, y, yaw);
//       }
//       catch (const tf::TransformException& ex)
//       {
//         ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
//       }
//     }

//     void connectedconvexhullsCb(const std_msgs::Float32MultiArray::ConstPtr &msg, int robot_id) {
//       if (robot_id < 1 || robot_id > num_robots_) return;
//       if (!allOhterPosesReady()) return;

//       const auto& data = msg->data;
//       if (data.size() % 2 != 0)
//       {
//         ROS_WARN("StarConvexHandler: data size not even, ignore.");
//         return;
//       }

//       std::vector<Eigen::Vector2d> tmp;
//       tmp.reserve(data.size()/2);
//       for (size_t i = 0; i < data.size(); i += 2)
//       {
//         if (!std::isfinite(data[i]) || !std::isfinite(data[i + 1])) {
//           ROS_WARN("StarConvexHandler: convex hull contains non-finite value, ignore.");
//           return;
//         }
//         tmp.emplace_back(static_cast<double>(data[i]),
//                          static_cast<double>(data[i + 1]));
//       }

//       {
//         std::lock_guard<std::mutex> lock(mtx_);
//         connected_robots_convexhull_by_id_[robot_id] = std::move(tmp);
//         has_connected_convexhulls_by_id_[robot_id] = true;
//       }
//     }

//     bool allOhterPosesReady() const {
//       std::lock_guard<std::mutex> lock(mtx_);
//       for (int id = 1; id <= num_robots_; ++id) {
//         if (id == robot_index_) continue;
//         if (id < 1 || id >= static_cast<int>(has_other_pose_by_id_.size())) return false;
//         if (!has_other_pose_by_id_[id]) return false;
//       }
//       return true;
//     }

//     bool allConnectedPosesReady() const {
//       std::lock_guard<std::mutex> lock(mtx_);
//       for (int nb : neighbors_) {
//         if (nb < 1 || nb >= static_cast<int>(has_connected_robots_pose_by_id_.size())) return false;
//         if (!has_connected_robots_pose_by_id_[nb]) return false;
//       }
//       return true;
//     }

//     bool allConnectedConvexHullsReady() const {
//       std::lock_guard<std::mutex> lock(mtx_);
//       for (int nb : neighbors_) {
//         if (nb < 1 || nb >= static_cast<int>(has_connected_convexhulls_by_id_.size())) return false;
//         if (!has_connected_convexhulls_by_id_[nb]) return false;
//       }
//       return true;
//     }

//     void edgesCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
//       if (msg->data.size() % 3 != 0 || robot_ns_.empty()) {
//         ROS_WARN_THROTTLE(1.0, "edges array size invalid (%zu), ignore", msg->data.size());
//         return;
//       }

//       std::vector<int> new_neighbors;
//       std::vector<float> new_weights;

//       for (size_t i = 0; i < msg->data.size(); i += 3) {
//         const float uf = msg->data[i];
//         const float vf = msg->data[i + 1];
//         const float w  = msg->data[i + 2];

//         if (!std::isfinite(uf) || !std::isfinite(vf) || !std::isfinite(w)) {
//           ROS_WARN_THROTTLE(1.0, "Non-finite edge data detected, ignore.");
//           continue;
//         }

//         int u = static_cast<int>(uf);
//         int v = static_cast<int>(vf);

//         if (u < 1 || u > num_robots_ || v < 1 || v > num_robots_) {
//           ROS_WARN_THROTTLE(1.0, "Invalid edge id: (%d, %d), num_robots=%d", u, v, num_robots_);
//           continue;
//         }

//         if (u == robot_index_) {
//           new_neighbors.push_back(v);
//           new_weights.push_back(w);
//         }
//         else if (v == robot_index_) {
//           new_neighbors.push_back(u);
//           new_weights.push_back(w);
//         }
//       }

//       std::set<int> old_set;
//       std::set<int> new_set(new_neighbors.begin(), new_neighbors.end());

//       {
//         std::lock_guard<std::mutex> lock(mtx_);
//         have_graph_ = true;
//         old_set = std::set<int>(neighbors_.begin(), neighbors_.end());

//         for (int id : old_set) {
//           if (new_set.count(id)) continue;
//           if (id < 1 || id > num_robots_) continue;

//           subs_connected_robots_pose_by_id_[id].shutdown();
//           subs_connected_convexhulls_by_id_[id].shutdown();
//           has_connected_robots_pose_by_id_[id] = false;
//           has_connected_convexhulls_by_id_[id] = false;
//           connected_robots_pose_by_id_[id] = Eigen::Vector2d::Zero();
//           connected_robots_convexhull_by_id_[id].clear();
//         }

//         for (int id : new_set) {
//           if (old_set.count(id)) continue;
//           if (id < 1 || id > num_robots_) continue;

//           subs_connected_robots_pose_by_id_[id] = nh_.subscribe<nav_msgs::Odometry>(
//             "/robot_" + std::to_string(id) + "/RosAria/odom", 1,
//             boost::bind(&PlanManager::connectedrobotsCb, this, _1, id)
//           );
//           subs_connected_convexhulls_by_id_[id] = nh_.subscribe<std_msgs::Float32MultiArray>(
//             "/robot_" + std::to_string(id) + "/vertices_convexhull", 1,
//             boost::bind(&PlanManager::connectedconvexhullsCb, this, _1, id)
//           );
//           has_connected_robots_pose_by_id_[id] = false;
//           has_connected_convexhulls_by_id_[id] = false;
//           connected_robots_pose_by_id_[id] = Eigen::Vector2d::Zero();
//           connected_robots_convexhull_by_id_[id].clear();
//         }

//         neighbors_ = std::move(new_neighbors);
//         weights_ = std::move(new_weights);
//       }

//       // std::ostringstream oss;
//       // oss << "robot " << robot_ns_ << " neighbors: ";
//       // bool first = true;
//       // for (int nb : neighbors_) {
//       //     if (!first) oss << ", ";
//       //     oss << nb;
//       //     first = false;
//       // }
//       // ROS_INFO_STREAM_THROTTLE(1.0, oss.str());
//     }

//     std::vector<int> getNeighbors() const {
//       std::lock_guard<std::mutex> lock(mtx_);
//       return std::vector<int>(neighbors_.begin(), neighbors_.end());
//     }

//     std::vector<float> getWeights() const {
//       std::lock_guard<std::mutex> lock(mtx_);
//       return weights_;
//     }

//     void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
//       // Ignore the given goal at runtime, commenting out this check may cause unexpected bugs
//       // Especially when there is no re-planning
//       // if(state_machine_ != StateMachine::IDLE){
//       //   ROS_ERROR("Haven't reached the goal yet!!");
//       //   return;
//       // }
//       ROS_INFO("\n\n\n\n\n\n\n\n");
//       ROS_INFO("---------------------------------------------------------------");
//       ROS_INFO("---------------------------------------------------------------");

//       ROS_INFO("get goal!");

//       {
//         std::lock_guard<std::mutex> lock(mtx_);
//         state_machine_ = StateMachine::IDLE;
//         have_goal_ = true;
//         goal_state_ << msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation);
//         if(if_fix_final_) goal_state_ = final_state_;
//       }

//       ROS_INFO_STREAM("goal state: " << goal_state_.transpose());

//       ROS_INFO("---------------------------------------------------------------");
//       ROS_INFO("---------------------------------------------------------------");
//       ROS_INFO("\n\n\n\n\n\n\n\n");
//     }

//     bool CheckCollisionWithRobots() {
//       std::lock_guard<std::mutex> lock(mtx_);
//       for (int id = 1; id <= num_robots_; ++id) {
//         if (id == robot_index_) continue;
//         if (id < 1 || id >= static_cast<int>(has_other_pose_by_id_.size())) continue;
//         if (!has_other_pose_by_id_[id]) continue;
//         if (hypot(current_state_XYTheta_.x() - other_robots_pose_by_id_[id].x(),
//                   current_state_XYTheta_.y() - other_robots_pose_by_id_[id].y()) < 1.0) {
//           return true;
//         }
//       }
//       return false;
//     }

//     void MainThread(const ros::TimerEvent& event){
//       (void)event;

//       bool have_geometry_local = false;
//       bool have_goal_local = false;
//       bool target_type_local = false;
//       bool graph_emergency_local = false;
//       StateMachine state_machine_local = StateMachine::INIT;
//       Eigen::Vector3d current_state_XYTheta_local = Eigen::Vector3d::Zero();
//       Eigen::Vector3d current_state_VAJ_local = Eigen::Vector3d::Zero();
//       Eigen::Vector3d current_state_OAJ_local = Eigen::Vector3d::Zero();
//       Eigen::Vector3d goal_state_local = Eigen::Vector3d::Zero();

//       std::vector<int> neighbors_local;
//       std::vector<Eigen::Vector2d> connected_robots_poses_;
//       std::vector<std::vector<Eigen::Vector2d>> connected_robots_convexhull_;
//       std::vector<Eigen::Vector2d> other_robots_poses_;

//       {
//         std::lock_guard<std::mutex> lock(mtx_);

//         have_geometry_local = have_geometry_;
//         have_goal_local = have_goal_;
//         target_type_local = target_type_;
//         graph_emergency_local = graph_emergency_;
//         state_machine_local = state_machine_;
//         current_state_XYTheta_local = current_state_XYTheta_;
//         current_state_VAJ_local = current_state_VAJ_;
//         current_state_OAJ_local = current_state_OAJ_;
//         goal_state_local = goal_state_;

//         if(!have_geometry_local || !have_goal_local) return;

//         for (int id = 1; id <= num_robots_; ++id) {
//           if (id == robot_index_) continue;
//           if (id < 1 || id >= static_cast<int>(has_other_pose_by_id_.size())) return;
//           if (!has_other_pose_by_id_[id]) return;
//         }

//         for (int nb : neighbors_) {
//           if (nb < 1 || nb > num_robots_) return;
//           if (nb >= static_cast<int>(has_connected_robots_pose_by_id_.size())) return;
//           if (nb >= static_cast<int>(has_connected_convexhulls_by_id_.size())) return;
//           if (!has_connected_robots_pose_by_id_[nb]) return;
//           if (!has_connected_convexhulls_by_id_[nb]) return;
//         }

//         neighbors_local = neighbors_;

//         connected_robots_poses_.reserve(neighbors_local.size());
//         connected_robots_convexhull_.reserve(neighbors_local.size());
//         for (int id : neighbors_local) {
//           connected_robots_poses_.push_back(connected_robots_pose_by_id_[id]);
//           connected_robots_convexhull_.push_back(connected_robots_convexhull_by_id_[id]);
//         }

//         other_robots_poses_.reserve(std::max(0, num_robots_ - 1));
//         for (int id = 1; id <= num_robots_; ++id) {
//           if (id == robot_index_) continue;
//           other_robots_poses_.push_back(other_robots_pose_by_id_[id]);
//         }
//       }

//       // collision check
//       if(have_geometry_local){
//         if(sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_local.x(), current_state_XYTheta_local.y())) < 0.0) {
//           std_msgs::Bool emergency_stop;
//           emergency_stop.data = true;
//           emergency_stop_pub_.publish(emergency_stop);

//           {
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = EMERGENCY_STOP;
//           }

//           ROS_INFO_STREAM(robot_ns_ + "'s current_state_XYTheta_: " << current_state_XYTheta_local.transpose());
//           ROS_INFO_STREAM("Dis: " << sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_local.x(), current_state_XYTheta_local.y())));
//           ROS_ERROR("EMERGENCY_STOP!!! Too close to obstacle!!!");
//           return;
//         }
//         // if(CheckCollisionWithRobots()){
//         //   std_msgs::Bool emergency_stop;
//         //   emergency_stop.data = true;
//         //   emergency_stop_pub_.publish(emergency_stop);
//         //   state_machine_ = EMERGENCY_STOP;
//         //   ROS_INFO_STREAM(robot_ns_ + "'s current_state_XYTheta_: " << current_state_XYTheta_.transpose());
//         //   ROS_ERROR("EMERGENCY_STOP!!! Too close to the other robots!!!");
//         //   return;
//         // }
//         // if(hypot(goal_state_(0)-current_state_XYTheta_(0), goal_state_(1)-current_state_XYTheta_(1)) < 0.3){
//         //   std_msgs::Bool emergency_stop;
//         //   emergency_stop.data = true;
//         //   emergency_stop_pub_.publish(emergency_stop);
//         //   state_machine_ = EMERGENCY_STOP;
//         //   ROS_INFO_STREAM(robot_ns_ + "'s current_state_XYTheta_: " << current_state_XYTheta_.transpose());
//         //   ROS_ERROR("Too close to the goal!!! No need to replan!!!");
//         //   return;
//         // }
//       }

//       if(state_machine_local == StateMachine::IDLE ||
//           ((state_machine_local == StateMachine::PLANNING || state_machine_local == StateMachine::REPLAN)
//             && (ros::Time::now() - loop_start_time_).toSec() > replan_time_)) {

//         loop_start_time_ = ros::Time::now();
//         double current = loop_start_time_.toSec();

//         // start new plan
//         if(state_machine_local == StateMachine::IDLE){
//           {
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = StateMachine::PLANNING;
//           }
//           plan_start_time_ = -1;
//           predicted_traj_start_time_ = -1;
//           plan_start_state_XYTheta = current_state_XYTheta_local;
//           plan_start_state_VAJ = current_state_VAJ_local;
//           plan_start_state_OAJ = current_state_OAJ_local;
//         }
//         // Use predicted distance for replanning in planning state
//         else if(state_machine_local == StateMachine::PLANNING || state_machine_local == StateMachine::REPLAN){

//           if(((current_state_XYTheta_local - goal_state_local).head(2).squaredNorm()
//               + fmod(fabs((plan_start_state_XYTheta - goal_state_local)[2]), 2.0 * M_PI) * 0.02 < 1.0) ||
//              msplanner_->final_traj_.getTotalDuration() < max_replan_time_){
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = StateMachine::GOINGTOGOAL;
//             return;
//           }

//           {
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = StateMachine::REPLAN;
//           }

//           predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
//           msplanner_->get_the_predicted_state(predicted_traj_start_time_, plan_start_state_XYTheta, plan_start_state_VAJ, plan_start_state_OAJ);
//         }

//         ROS_INFO("\033[32;40m \n\n\n\n\n-------------------------------------start new plan------------------------------------------ \033[0m");

//         visualizer_->finalnodePub(plan_start_state_XYTheta, goal_state_local);
//         ROS_INFO("init_state_: %.10f  %.10f  %.10f", plan_start_state_XYTheta(0), plan_start_state_XYTheta(1), plan_start_state_XYTheta(2));
//         ROS_INFO("goal_state_: %.10f  %.10f  %.10f", goal_state_local(0), goal_state_local(1), goal_state_local(2));
//         std::cout << "<arg name=\"start_x_\" value=\"" << plan_start_state_XYTheta(0) << "\"/>" << std::endl;
//         std::cout << "<arg name=\"start_y_\" value=\"" << plan_start_state_XYTheta(1) << "\"/>" << std::endl;
//         std::cout << "<arg name=\"start_yaw_\" value=\"" << plan_start_state_XYTheta(2) << "\"/>" << std::endl;
//         std::cout << "<arg name=\"final_x_\" value=\"" << goal_state_local(0) << "\"/>" << std::endl;
//         std::cout << "<arg name=\"final_y_\" value=\"" << goal_state_local(1) << "\"/>" << std::endl;
//         std::cout << "<arg name=\"final_yaw_\" value=\"" << goal_state_local(2) << "\"/>" << std::endl;

//         std::cout << "plan_start_state_VAJ: " << plan_start_state_VAJ.transpose() << std::endl;
//         std::cout << "plan_start_state_OAJ: " << plan_start_state_OAJ.transpose() << std::endl;

//         ROS_INFO("<arg name=\"start_x_\" value=\"%f\"/>", plan_start_state_XYTheta(0));
//         ROS_INFO("<arg name=\"start_y_\" value=\"%f\"/>", plan_start_state_XYTheta(1));
//         ROS_INFO("<arg name=\"start_yaw_\" value=\"%f\"/>", plan_start_state_XYTheta(2));
//         ROS_INFO("<arg name=\"final_x_\" value=\"%f\"/>", goal_state_local(0));
//         ROS_INFO("<arg name=\"final_y_\" value=\"%f\"/>", goal_state_local(1));
//         ROS_INFO("<arg name=\"final_yaw_\" value=\"%f\"/>", goal_state_local(2));

//         ROS_INFO_STREAM("plan_start_state_VAJ: " << plan_start_state_VAJ.transpose());
//         ROS_INFO_STREAM("plan_start_state_OAJ: " << plan_start_state_OAJ.transpose());

//         // front end
//         ros::Time astar_start_time = ros::Time::now();
//         if(!findJPSRoad()){
//           {
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = EMERGENCY_STOP;
//           }
//           ROS_ERROR("EMERGENCY_STOP!!! can not find astar road !!!");
//           return;
//         }
//         ROS_INFO("\033[41;37m all of front end time:%f \033[0m", (ros::Time::now() - astar_start_time).toSec());

//         // optimizer
//         double scale_down_param = 1.0;
//         std_msgs::Float64 msg;
//         std::vector<Eigen::Vector3d> scp_info(connected_robots_poses_.size());
//         for (int i = 0; i < static_cast<int>(connected_robots_poses_.size()); i++) {
//           scp_info[i] = scp_->starConvexConstraint(
//             {current_state_XYTheta_local.x(), current_state_XYTheta_local.y()},
//             connected_robots_poses_[i],
//             connected_robots_convexhull_[i],
//             false);
//           ROS_INFO("%s: Cost: %f, gradient: (%f, %f)",
//                    robot_ns_.c_str(), scp_info[i][0], scp_info[i][1], scp_info[i][2]);
//         }

//         double min_los = std::numeric_limits<double>::infinity();
//         if (target_type_local) {
//           for (int i = 0; i < static_cast<int>(connected_robots_poses_.size()); i++) {
//             auto temp_los = scp_info[i][0];
//             temp_los = (0.0 - temp_los);
//             if (temp_los < min_los) {
//               min_los = temp_los;
//             }
//           }
//         const int STOP_ROUNDS = 10;

//         // 1) trigger latch
//         if (((min_los + 2.5 <= los_margin_) || graph_emergency_) && stop_countdown_ == 0) {
//             stop_countdown_ = STOP_ROUNDS;
//         }

//         // 2) if latched -> keep stopping
//         if (stop_countdown_ > 0) {
//             {
//                 std::lock_guard<std::mutex> lock(mtx_);
//                 state_machine_ = EMERGENCY_STOP;
//             }

//             std_msgs::Bool emergency_stop;
//             emergency_stop.data = true;
//             emergency_stop_pub_.publish(emergency_stop);

//             ROS_ERROR("EMERGENCY_STOP!!! %s min_los=%.3f (th=%.3f) stop_countdown=%d",
//                       robot_ns_.c_str(), min_los+2.5, los_margin_, stop_countdown_);

//             stop_countdown_--;
//             return;
//         }

//         // 3) release (publish false once if you want)
//         {
//             std_msgs::Bool emergency_stop;
//             emergency_stop.data = false;
//             emergency_stop_pub_.publish(emergency_stop);
//         }
//           if (min_los + 2.5 <= 6.0) {
//             scale_down_param = 1 - exp(-alpha_ * min_los);
//             scale_down_param = std::max(sd_threshold_, scale_down_param);
//           }
//           else {
//             scale_down_param = 1.0;
//           }
//           ROS_INFO("Scale down param: %f", scale_down_param);
//         }
//         else {
//           for (int i = 0; i < static_cast<int>(connected_robots_poses_.size()); i++) {
//             auto temp_los = scp_info[i][0];
//             temp_los = (0.0 - temp_los);
//             if (temp_los < min_los) {
//               min_los = temp_los;
//             }
//           }
//           // --- latch trigger: if min_los below margin, start/refresh countdown ---
//           if (min_los + 2.5 <= (los_margin_)) {
//             // 触发一次就锁 10 轮；如果你想“持续低于阈值就一直续命”，用 max() 保持不变
//             // los_stop_countdown_ = std::max(los_stop_countdown_, los_stop_rounds_);
//             los_stop_countdown_ = los_stop_rounds_;
//           }
          
//           // --- while countdown > 0: keep emergency true ---
//           if (los_stop_countdown_ > 0) {
//             std_msgs::Bool solving_failed;
//             solving_failed.data = true;
//             emergency_stop_pub_.publish(solving_failed);
//             ROS_ERROR("EMERGENCY_STOP!!! %s min_los=%.3f (th=%.3f) stop_countdown=%d",
//                       robot_ns_.c_str(), min_los+2.5, los_margin_, stop_countdown_);
//             solving_failed_pub_.publish(solving_failed);
//             graph_emergency_param_pub_.publish(solving_failed);

//             // 如果你还想保留 emerge_stop_counts 做日志/统计，可以这样写：
//             emerge_stop_counts = los_stop_rounds_ - los_stop_countdown_ + 1;

//             los_stop_countdown_--;
//           } else {
//             // countdown finished -> release (publish false once is better, but simplest is always publish)
//             std_msgs::Bool solving_failed;
//             solving_failed.data = false;
//             emergency_stop_pub_.publish(solving_failed);
//             solving_failed_pub_.publish(solving_failed);
//             graph_emergency_param_pub_.publish(solving_failed);
//             emerge_stop_counts = 0;
//           }
//         }

//         msg.data = scale_down_param;
//         scale_down_param_pub_.publish(msg);
//         msplanner_->set_scale_down_param(scale_down_param);
//         msplanner_->set_other_robots_states(other_robots_poses_);
//         msplanner_->set_connected_robots_states(connected_robots_poses_);
//         msplanner_->set_connected_robots_convexhulls(connected_robots_convexhull_);
//         msplanner_->set_connected_robots_scps(scp_info);

//         bool result = msplanner_->minco_plan(jps_planner_->flat_traj_);
//         std_msgs::Bool solving_failed;
//         if(!result){
//           {
//             std::lock_guard<std::mutex> lock(mtx_);
//             state_machine_ = EMERGENCY_STOP;
//           }
//           ROS_ERROR("EMERGENCY_STOP! Fail to find a feasible solution!!!");
//           std_msgs::Bool emergency_stop;
//           emergency_stop.data = true;
//           emergency_stop_pub_.publish(emergency_stop);
//           return;
//         }
//         else {
//           if (target_type_local) {
//             solving_failed.data = false;
//             solving_failed_pub_.publish(solving_failed);
//           }
//         }
//         // else if (result && !CheckTrajFeasibility()) {
//         //   ROS_ERROR("Trajectory intersects with the obs! Infeasible Traj!");
//         //   std_msgs::Bool infeasible_traj;
//         //   infeasible_traj.data = true;
//         //   traj_feasibility_pub_.publish(infeasible_traj);
//         //   return;
//         // }

//         ROS_INFO("\033[43;32m all of plan time:%f \033[0m", (ros::Time::now().toSec() - current));

//         // visualization
//         msplanner_->mincoPathPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPathPath);
//         msplanner_->mincoPointPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPointMarker, Eigen::Vector3d(239, 41, 41));

//         // for replan
//         if(plan_start_time_ < 0){
//           Traj_start_time_ = ros::Time::now();
//           plan_start_time_ = Traj_start_time_.toSec();
//         }
//         else{
//           plan_start_time_ = current + max_replan_time_;
//           Traj_start_time_ = ros::Time(plan_start_time_);
//         }

//         MPCPathPub(plan_start_time_);
//         Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
//       }

//       if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_){
//         std::lock_guard<std::mutex> lock(mtx_);
//         state_machine_ = StateMachine::IDLE;
//         have_goal_ = false;
//       }
//     }

//     bool findJPSRoad(){
//       jps_planner_->set_scale_down_param(msplanner_->get_scale_down_param());
//       ros::Time current = ros::Time::now();
//       Eigen::Vector3d start_state;
//       std::vector<Eigen::Vector3d> start_path;
//       std::vector<Eigen::Vector3d> start_path_both_end;
//       bool if_forward = true;

//       if(plan_start_time_ > 0){
//         start_path = msplanner_->get_the_predicted_state_and_path(
//           predicted_traj_start_time_,
//           predicted_traj_start_time_ + jps_planner_->jps_truncation_time_,
//           plan_start_state_XYTheta,
//           start_state,
//           if_forward);

//         u_int start_path_size = start_path.size();
//         u_int start_path_i = 0;
//         for(; start_path_i < start_path_size; start_path_i++){
//           if(!jps_planner_->JPS_check_if_collision(start_path[start_path_i].head(2)))
//             break;
//         }

//         if (start_path.empty()) {
//           start_state = plan_start_state_XYTheta;
//         }
//         else if(start_path_i == 0){
//           start_state = plan_start_state_XYTheta;
//           start_path_both_end.push_back(start_path.front());
//           start_path_both_end.push_back(start_state);
//         }
//         else if(start_path_i < start_path_size){
//           start_path = std::vector<Eigen::Vector3d>(start_path.begin(), start_path.begin() + start_path_i);
//           start_state = start_path.back();
//           start_path_both_end.push_back(start_path.front());
//           start_path_both_end.push_back(start_state);
//         }
//         else{
//           start_path_both_end.push_back(start_path.front());
//           start_path_both_end.push_back(start_state);
//         }
//       }
//       else{
//         start_state = plan_start_state_XYTheta;
//       }

//       // start_state = plan_start_state_XYTheta;
//       jps_planner_->plan(start_state, goal_state_);
//       jps_planner_->getKinoNodeWithStartPath(start_path, if_forward, plan_start_state_VAJ, plan_start_state_OAJ);

//       visualization_msgs::Marker marker;
//       marker.header.frame_id = "map";
//       marker.header.stamp = ros::Time::now();
//       marker.ns = "jps_planner";
//       marker.id = 0;
//       marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//       marker.action = visualization_msgs::Marker::ADD;
//       marker.pose.position.x = 11;
//       marker.pose.position.y = 8;
//       marker.pose.position.z = 0;
//       marker.pose.orientation.x = 0.0;
//       marker.pose.orientation.y = 0.0;
//       marker.pose.orientation.z = 0.0;
//       marker.pose.orientation.w = 1.0;
//       marker.scale.z = 0.5;
//       marker.color.a = 1.0; // Don't forget to set the alpha!
//       marker.color.r = 0.0;
//       marker.color.g = 0.0;
//       marker.color.b = 0.0;
//       double search_time = (ros::Time::now() - current).toSec() * 1000.0;
//       std::ostringstream out;
//       out << std::fixed << "JPS: \n" << std::setprecision(2) << search_time << " ms";
//       marker.text = out.str();
//       record_pub_.publish(marker);

//       ROS_INFO("\033[40;36m jps_planner_ search time:%lf  \033[0m", (ros::Time::now() - current).toSec());
//       return true;
//     }

//     void MPCPathPub(const double& traj_start_time){
//       (void)traj_start_time;

//       Eigen::MatrixXd initstate = msplanner_->get_current_iniState();
//       Eigen::MatrixXd finState = msplanner_->get_current_finState();
//       Eigen::MatrixXd finalInnerpoints = msplanner_->get_current_Innerpoints();
//       Eigen::VectorXd finalpieceTime = msplanner_->get_current_finalpieceTime();
//       Eigen::Vector3d iniStateXYTheta = msplanner_->get_current_iniStateXYTheta();

//       carstatemsgs::Polynome polynome;
//       polynome.header.frame_id = "map";
//       polynome.header.stamp = ros::Time::now();
//       polynome.init_p.x = initstate.col(0).x();
//       polynome.init_p.y = initstate.col(0).y();
//       polynome.init_v.x = initstate.col(1).x();
//       polynome.init_v.y = initstate.col(1).y();
//       polynome.init_a.x = initstate.col(2).x();
//       polynome.init_a.y = initstate.col(2).y();
//       polynome.tail_p.x = finState.col(0).x();
//       polynome.tail_p.y = finState.col(0).y();
//       polynome.tail_v.x = finState.col(1).x();
//       polynome.tail_v.y = finState.col(1).y();
//       polynome.tail_a.x = finState.col(2).x();
//       polynome.tail_a.y = finState.col(2).y();

//       if(plan_start_time_ < 0) polynome.traj_start_time = ros::Time::now();
//       else polynome.traj_start_time = ros::Time(plan_start_time_);

//       for(u_int i = 0; i < finalInnerpoints.cols(); i++){
//         geometry_msgs::Vector3 point;
//         point.x = finalInnerpoints.col(i).x();
//         point.y = finalInnerpoints.col(i).y();
//         point.z = 0.0;
//         polynome.innerpoints.push_back(point);
//       }
//       for(u_int i = 0; i < finalpieceTime.size(); i++){
//         polynome.t_pts.push_back(finalpieceTime[i]);
//       }
//       polynome.start_position.x = iniStateXYTheta.x();
//       polynome.start_position.y = iniStateXYTheta.y();
//       polynome.start_position.z = iniStateXYTheta.z();

//       if(!msplanner_->if_standard_diff_){
//         polynome.ICR.x = msplanner_->ICR_.x();
//         polynome.ICR.y = msplanner_->ICR_.y();
//         polynome.ICR.z = msplanner_->ICR_.z();
//       }

//       mpc_polynome_pub_.publish(polynome);
//     }

//     // bool CheckTrajFeasibility(){
//     //   Eigen::MatrixXd initstate = msplanner_->get_current_iniState();
//     //   Eigen::MatrixXd finState = msplanner_->get_current_finState();
//     //   Eigen::MatrixXd finalInnerpoints = msplanner_->get_current_Innerpoints();
//     //   Eigen::Vector3d iniStateXYTheta = msplanner_->get_current_iniStateXYTheta();

//     //   auto sx = initstate.col(0).x();
//     //   auto sy = initstate.col(0).y();
//     //   auto ex = finState.col(0).x();
//     //   auto ey = finState.col(0).y();
//     //   if (sdfmap_->getDistanceReal({sx, sy}) < 0.0 || sdfmap_->getDistanceReal({ex, ey}) < 0.0) return false;
//     //   for(u_int i=0; i<finalInnerpoints.cols(); i++){
//     //     if(sdfmap_->getDistanceReal({finalInnerpoints.col(i).x(), finalInnerpoints.col(i).y()}) < 0.0) return false;
//     //   }
//     //   return true;
//     // }

// };

// #endif

#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include "plan_env/sdf_map.h"
#include "visualizer/visualizer.hpp"
#include "front_end/jps_planner/jps_planner.h"
#include "geometry_msgs/PoseStamped.h"
#include "back_end/optimizer.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "carstatemsgs/CarState.h"
#include "carstatemsgs/Polynome.h"
#include "std_msgs/Bool.h"
#include "star_convex_handle.h"

#include <thread>
#include <mutex>
#include <set>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <stdexcept>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

enum StateMachine{
  INIT,
  IDLE,
  PLANNING,
  REPLAN,
  GOINGTOGOAL,
  EMERGENCY_STOP,
};

class PlanManager
{
  private:
    ros::NodeHandle nh_;

    std::string name_space;
    std::shared_ptr<SDFmap> sdfmap_;
    std::shared_ptr<Visualizer> visualizer_;
    std::shared_ptr<MSPlanner> msplanner_;
    std::shared_ptr<JPS::JPSPlanner> jps_planner_;

    ros::Subscriber goal_sub_;
    ros::Subscriber current_state_sub_, current_vel_sub_;
    ros::Subscriber sub_target_temp_, sub_graph_emergency_;
    ros::Subscriber edges_sub_;
    ros::Subscriber sub_weights_;
    ros::Subscriber current_state_pose_sub_;
    ros::Timer main_thread_timer_;
    ros::Publisher cmd_pub_;
    ros::Publisher mpc_polynome_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher solving_failed_pub_;
    ros::Publisher traj_feasibility_pub_;
    ros::Publisher scale_down_param_pub_, graph_emergency_param_pub_;

    ros::Publisher record_pub_;

    ros::Time current_time_;
    Eigen::Vector3d current_state_XYTheta_;
    Eigen::Vector3d current_state_VAJ_;
    Eigen::Vector3d current_state_OAJ_;

    double plan_start_time_;
    Eigen::Vector3d plan_start_state_XYTheta;
    Eigen::Vector3d plan_start_state_VAJ;
    Eigen::Vector3d plan_start_state_OAJ;

    Eigen::Vector3d goal_state_;

    ros::Time Traj_start_time_;
    double Traj_total_time_;

    ros::Time loop_start_time_;

    bool have_geometry_ = false;
    bool have_graph_ = false;
    bool have_goal_ = false;
    bool has_target_type_ = false;
    bool target_type_ = false;
    bool graph_emergency_ = false;

    bool if_fix_final_ = false;
    Eigen::Vector3d final_state_ = Eigen::Vector3d::Zero();

    double replan_time_ = 10000.0;
    double max_replan_time_ = 1.0;

    double predicted_traj_start_time_ = -1.0;

    StateMachine state_machine_ = StateMachine::INIT;

    std::string robot_ns_;
    int robot_index_ = 0;
    mutable std::mutex mtx_;
    std::vector<int> neighbors_;
    std::vector<float> weights_;
    int num_robots_ = 0;
    double los_margin_, alpha_, sd_threshold_;
    std::vector<Eigen::Vector2d> other_robots_pose_by_id_;
    std::vector<bool> has_other_pose_by_id_;
    std::vector<ros::Subscriber> subs_other_robots_pose_by_id_;

    std::vector<Eigen::Vector2d> connected_robots_pose_by_id_;
    std::vector<bool> has_connected_robots_pose_by_id_;
    std::vector<std::vector<Eigen::Vector2d>> connected_robots_convexhull_by_id_;
    std::vector<bool> has_connected_convexhulls_by_id_;
    std::vector<ros::Subscriber> subs_connected_robots_pose_by_id_;
    std::vector<ros::Subscriber> subs_connected_convexhulls_by_id_;

    std::shared_ptr<star_convex_handle::StarConvexHandler> scp_;

    // ===== LoS emergency-stop latch (FIXED) =====
    // 目标：一旦触发，强制停 STOP_ROUNDS 轮，并在这 STOP_ROUNDS 内持续 publish emergency_stop=true
    // 不把 state_machine_ 切到 EMERGENCY_STOP（否则你那段逻辑后续 tick 进不来，倒计时卡死）
    int los_stop_countdown_{0};         // remaining stop rounds
    bool los_stop_active_{false};       // last published emergency_stop state
    int los_stop_rounds_{10};           // default stop rounds (can param)
    double los_bias_{3.5};              // 你现在用的 +2.5 偏置

  private:
    int parseRobotIndex(const std::string& ns) {
      std::size_t pos = ns.rfind("robot_");
      if (pos == std::string::npos) {
        ROS_WARN("Failed to parse robot index from namespace: %s", ns.c_str());
        return 0;
      }
      pos += 6;
      std::size_t end = pos;
      while (end < ns.size() && std::isdigit(static_cast<unsigned char>(ns[end]))) ++end;
      if (end == pos) {
        ROS_WARN("Failed to parse robot index digits from namespace: %s", ns.c_str());
        return 0;
      }
      try {
        return std::stoi(ns.substr(pos, end - pos));
      } catch (...) {
        ROS_WARN("Exception when parsing robot index from namespace: %s", ns.c_str());
        return 0;
      }
    }

  public:
    PlanManager(ros::NodeHandle nh){
      nh_ = nh;
      scp_ = std::make_shared<star_convex_handle::StarConvexHandler>(nh_);

      nh_.param("num_robots", num_robots_, 0);
      nh_.param("los_margin", los_margin_, 1.5);
      nh_.param("alpha", alpha_, 0.25);
      nh_.param("sd_threshold", sd_threshold_, 0.6);

      // LoS stop params (optional)
      nh_.param("los_stop_rounds", los_stop_rounds_, 10);
      nh_.param("los_bias", los_bias_, 3.5);

      other_robots_pose_by_id_.assign(num_robots_ + 1, Eigen::Vector2d::Zero());
      has_other_pose_by_id_.assign(num_robots_ + 1, false);
      subs_other_robots_pose_by_id_.resize(num_robots_ + 1);

      connected_robots_pose_by_id_.assign(num_robots_ + 1, Eigen::Vector2d::Zero());
      has_connected_robots_pose_by_id_.assign(num_robots_ + 1, false);
      connected_robots_convexhull_by_id_.resize(num_robots_ + 1);
      has_connected_convexhulls_by_id_.assign(num_robots_ + 1, false);
      subs_connected_robots_pose_by_id_.resize(num_robots_ + 1);
      subs_connected_convexhulls_by_id_.resize(num_robots_ + 1);

      sdfmap_ = std::make_shared<SDFmap>(nh);
      visualizer_ = std::make_shared<Visualizer>(nh);
      msplanner_ = std::make_shared<MSPlanner>(Config(ros::NodeHandle("~")), nh_, sdfmap_);
      jps_planner_ = std::make_shared<JPS::JPSPlanner>(sdfmap_, nh_);

      robot_ns_ = ros::names::parentNamespace(nh_.getNamespace());
      robot_index_ = parseRobotIndex(robot_ns_);

      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        robot_ns_ + "/move_base_simple/goal", 1, &PlanManager::goal_callback, this);
      current_state_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        robot_ns_ + "/RosAria/odom", 10, &PlanManager::GeometryCallbackPose, this);
      sub_target_temp_ = nh_.subscribe<std_msgs::Bool>(
        robot_ns_ + "/target_type", 10, &PlanManager::CallbackTT, this);
      sub_graph_emergency_ = nh_.subscribe<std_msgs::Bool>(
        "/graph_emergency", 10, &PlanManager::CallbackGraphEmergency, this);

      for (int id = 1; id <= num_robots_; ++id) {
        if (id == robot_index_) continue;
        subs_other_robots_pose_by_id_[id] = nh_.subscribe<nav_msgs::Odometry>(
          "/robot_" + std::to_string(id) + "/RosAria/odom", 1,
          boost::bind(&PlanManager::otherrobotsCb, this, _1, id)
        );
      }

      edges_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>(
        "/mst_graph/edges", 1, &PlanManager::edgesCb, this);

      main_thread_timer_ = nh_.createTimer(
        ros::Duration(0.001), &PlanManager::MainThread, this);

      cmd_pub_ = nh_.advertise<carstatemsgs::CarState>(robot_ns_ + "/simulation/PoseSub", 1);
      emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/emergency_stop", 1);
      solving_failed_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/solving_failed", 1);
      traj_feasibility_pub_ = nh_.advertise<std_msgs::Bool>(robot_ns_ + "/planner/traj_feasibility", 1);

      record_pub_ = nh_.advertise<visualization_msgs::Marker>(robot_ns_ + "/planner/calculator_time", 1);

      scale_down_param_pub_ = nh_.advertise<std_msgs::Float64>(robot_ns_ + "/scale_down_pram", 1);
      graph_emergency_param_pub_ = nh_.advertise<std_msgs::Bool>("/graph_emergency", 1);
      mpc_polynome_pub_ = nh_.advertise<carstatemsgs::Polynome>(robot_ns_ + "/traj", 1);

      nh_.param<bool>("if_fix_final", if_fix_final_, false);
      if (if_fix_final_) {
        nh_.param<double>("final_x", final_state_(0), 0.0);
        nh_.param<double>("final_y", final_state_(1), 0.0);
        nh_.param<double>("final_yaw", final_state_(2), 0.0);
      }

      nh_.param<double>("replan_time", replan_time_, 10000.0);
      nh_.param<double>("max_replan_time", max_replan_time_, 1.0);

      state_machine_ = StateMachine::IDLE;
      loop_start_time_ = ros::Time::now();
    }

    ~PlanManager() = default;

    void CallbackTT(const std_msgs::Bool::ConstPtr &msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      has_target_type_ = true;
      target_type_ = msg->data;
    }

    void CallbackGraphEmergency(const std_msgs::Bool::ConstPtr &msg) {
      std::lock_guard<std::mutex> lock(mtx_);
      graph_emergency_ = msg->data;
    }

    void GeometryCallbackPose(const nav_msgs::Odometry::ConstPtr &msg){
      static tf2_ros::Buffer tf_buffer;
      static tf2_ros::TransformListener tf_listener(tf_buffer);
      try
      {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp);
        const double x = transformStamped.transform.translation.x;
        const double y = transformStamped.transform.translation.y;
        const double yaw = tf::getYaw(transformStamped.transform.rotation);

        {
          std::lock_guard<std::mutex> lock(mtx_);
          have_geometry_ = true;
          current_state_XYTheta_ << x, y, yaw;
          current_state_VAJ_ << msg->twist.twist.linear.x, 0.0, 0.0;
          current_state_OAJ_ << msg->twist.twist.angular.z, 0.0, 0.0;
          current_time_ = ros::Time::now();
        }
      }
      catch (const tf::TransformException& ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      }
    }

    void otherrobotsCb(const nav_msgs::Odometry::ConstPtr &msg, int robot_id) {
      if (robot_id < 1 || robot_id > num_robots_) return;

      static tf2_ros::Buffer tf_buffer;
      static tf2_ros::TransformListener tf_listener(tf_buffer);
      try
      {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp, ros::Duration(0.1));
        const double x = transformStamped.transform.translation.x;
        const double y = transformStamped.transform.translation.y;

        std::lock_guard<std::mutex> lock(mtx_);
        have_geometry_ = true;
        other_robots_pose_by_id_[robot_id] = Eigen::Vector2d(x, y);
        has_other_pose_by_id_[robot_id] = true;
      }
      catch (const tf::TransformException& ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      }
    }

    void connectedrobotsCb(const nav_msgs::Odometry::ConstPtr &msg, int robot_id) {
      if (robot_id < 1 || robot_id > num_robots_) return;

      static tf2_ros::Buffer tf_buffer;
      static tf2_ros::TransformListener tf_listener(tf_buffer);
      try
      {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer.lookupTransform("map", msg->child_frame_id, msg->header.stamp, ros::Duration(0.1));
        const double x = transformStamped.transform.translation.x;
        const double y = transformStamped.transform.translation.y;
        const double yaw = tf::getYaw(transformStamped.transform.rotation);
        (void)yaw;

        std::lock_guard<std::mutex> lock(mtx_);
        have_geometry_ = true;
        connected_robots_pose_by_id_[robot_id] = Eigen::Vector2d(x, y);
        has_connected_robots_pose_by_id_[robot_id] = true;
      }
      catch (const tf::TransformException& ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      }
    }

    void connectedconvexhullsCb(const std_msgs::Float32MultiArray::ConstPtr &msg, int robot_id) {
      if (robot_id < 1 || robot_id > num_robots_) return;
      if (!allOhterPosesReady()) return;

      const auto& data = msg->data;
      if (data.size() % 2 != 0)
      {
        ROS_WARN("StarConvexHandler: data size not even, ignore.");
        return;
      }

      std::vector<Eigen::Vector2d> tmp;
      tmp.reserve(data.size()/2);
      for (size_t i = 0; i < data.size(); i += 2)
      {
        if (!std::isfinite(data[i]) || !std::isfinite(data[i + 1])) {
          ROS_WARN("StarConvexHandler: convex hull contains non-finite value, ignore.");
          return;
        }
        tmp.emplace_back(static_cast<double>(data[i]),
                         static_cast<double>(data[i + 1]));
      }

      {
        std::lock_guard<std::mutex> lock(mtx_);
        connected_robots_convexhull_by_id_[robot_id] = std::move(tmp);
        has_connected_convexhulls_by_id_[robot_id] = true;
      }
    }

    bool allOhterPosesReady() const {
      std::lock_guard<std::mutex> lock(mtx_);
      for (int id = 1; id <= num_robots_; ++id) {
        if (id == robot_index_) continue;
        if (id < 1 || id >= static_cast<int>(has_other_pose_by_id_.size())) return false;
        if (!has_other_pose_by_id_[id]) return false;
      }
      return true;
    }

    void edgesCb(const std_msgs::Float32MultiArrayConstPtr& msg) {
      if (msg->data.size() % 3 != 0 || robot_ns_.empty()) {
        ROS_WARN_THROTTLE(1.0, "edges array size invalid (%zu), ignore", msg->data.size());
        return;
      }

      std::vector<int> new_neighbors;
      std::vector<float> new_weights;

      for (size_t i = 0; i < msg->data.size(); i += 3) {
        const float uf = msg->data[i];
        const float vf = msg->data[i + 1];
        const float w  = msg->data[i + 2];

        if (!std::isfinite(uf) || !std::isfinite(vf) || !std::isfinite(w)) {
          ROS_WARN_THROTTLE(1.0, "Non-finite edge data detected, ignore.");
          continue;
        }

        int u = static_cast<int>(uf);
        int v = static_cast<int>(vf);

        if (u < 1 || u > num_robots_ || v < 1 || v > num_robots_) {
          ROS_WARN_THROTTLE(1.0, "Invalid edge id: (%d, %d), num_robots=%d", u, v, num_robots_);
          continue;
        }

        if (u == robot_index_) {
          new_neighbors.push_back(v);
          new_weights.push_back(w);
        }
        else if (v == robot_index_) {
          new_neighbors.push_back(u);
          new_weights.push_back(w);
        }
      }

      std::set<int> old_set;
      std::set<int> new_set(new_neighbors.begin(), new_neighbors.end());

      {
        std::lock_guard<std::mutex> lock(mtx_);
        have_graph_ = true;
        old_set = std::set<int>(neighbors_.begin(), neighbors_.end());

        for (int id : old_set) {
          if (new_set.count(id)) continue;
          if (id < 1 || id > num_robots_) continue;

          subs_connected_robots_pose_by_id_[id].shutdown();
          subs_connected_convexhulls_by_id_[id].shutdown();
          has_connected_robots_pose_by_id_[id] = false;
          has_connected_convexhulls_by_id_[id] = false;
          connected_robots_pose_by_id_[id] = Eigen::Vector2d::Zero();
          connected_robots_convexhull_by_id_[id].clear();
        }

        for (int id : new_set) {
          if (old_set.count(id)) continue;
          if (id < 1 || id > num_robots_) continue;

          subs_connected_robots_pose_by_id_[id] = nh_.subscribe<nav_msgs::Odometry>(
            "/robot_" + std::to_string(id) + "/RosAria/odom", 1,
            boost::bind(&PlanManager::connectedrobotsCb, this, _1, id)
          );
          subs_connected_convexhulls_by_id_[id] = nh_.subscribe<std_msgs::Float32MultiArray>(
            "/robot_" + std::to_string(id) + "/vertices_convexhull", 1,
            boost::bind(&PlanManager::connectedconvexhullsCb, this, _1, id)
          );
          has_connected_robots_pose_by_id_[id] = false;
          has_connected_convexhulls_by_id_[id] = false;
          connected_robots_pose_by_id_[id] = Eigen::Vector2d::Zero();
          connected_robots_convexhull_by_id_[id].clear();
        }

        neighbors_ = std::move(new_neighbors);
        weights_ = std::move(new_weights);
      }
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
      ROS_INFO("\n\n\n\n\n\n\n\n");
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("get goal!");

      {
        std::lock_guard<std::mutex> lock(mtx_);
        state_machine_ = StateMachine::IDLE;
        have_goal_ = true;
        goal_state_ << msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation);
        if(if_fix_final_) goal_state_ = final_state_;
      }

      ROS_INFO_STREAM("goal state: " << goal_state_.transpose());
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("---------------------------------------------------------------");
      ROS_INFO("\n\n\n\n\n\n\n\n");
    }

    // ======================= REPLACE THIS WHOLE FUNCTION =======================
    void MainThread(const ros::TimerEvent& event){
      (void)event;

      // -------------------- local snapshots --------------------
      bool have_geometry_local = false;
      bool have_goal_local = false;
      bool target_type_local = false;
      bool graph_emergency_local = false;
      StateMachine state_machine_local = StateMachine::INIT;

      Eigen::Vector3d current_state_XYTheta_local = Eigen::Vector3d::Zero();
      Eigen::Vector3d current_state_VAJ_local     = Eigen::Vector3d::Zero();
      Eigen::Vector3d current_state_OAJ_local     = Eigen::Vector3d::Zero();
      Eigen::Vector3d goal_state_local            = Eigen::Vector3d::Zero();

      std::vector<int> neighbors_local;
      std::vector<Eigen::Vector2d> connected_robots_poses_;
      std::vector<std::vector<Eigen::Vector2d>> connected_robots_convexhull_;
      std::vector<Eigen::Vector2d> other_robots_poses_;

      // -------------------- (A0) ALWAYS keep latched stop alive, even if data not ready --------------------
      // Key idea: once triggered, we must keep publishing emergency_stop=true for los_stop_rounds_ ticks,
      // regardless of readiness checks below (poses/hulls/graph/goal etc.)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (los_stop_countdown_ > 0) {
          std_msgs::Bool emergency_stop;
          emergency_stop.data = true;
          emergency_stop_pub_.publish(emergency_stop);

          // keep EMERGENCY_STOP optional (you can comment this if it affects other modules)
          state_machine_ = EMERGENCY_STOP;

          ROS_ERROR_THROTTLE(0.2,
            "EMERGENCY_STOP(LATCH-KEEPALIVE) %s countdown=%d",
            robot_ns_.c_str(), los_stop_countdown_);

          los_stop_countdown_--;
          los_stop_active_ = true;
          return;
        }
      }

      // -------------------- (A1) READ SNAPSHOT (minimal) --------------------
      {
        std::lock_guard<std::mutex> lock(mtx_);
        have_geometry_local   = have_geometry_;
        have_goal_local       = have_goal_;
        target_type_local     = target_type_;
        graph_emergency_local = graph_emergency_;
        state_machine_local   = state_machine_;

        current_state_XYTheta_local = current_state_XYTheta_;
        current_state_VAJ_local     = current_state_VAJ_;
        current_state_OAJ_local     = current_state_OAJ_;
        goal_state_local            = goal_state_;
      }

      // -------------------- (A2) GRAPH EMERGENCY: trigger latch even if other data not ready --------------------
      // This guarantees: if /graph_emergency is true, robot WILL stop for los_stop_rounds_ ticks.
      if (graph_emergency_local) {
        {
          std::lock_guard<std::mutex> lock(mtx_);
          if (los_stop_countdown_ == 0) {
            los_stop_countdown_ = std::max(1, los_stop_rounds_);
            ROS_ERROR("\033[41;37m[GRAPH-EMG STOP TRIGGER] %s rounds=%d\033[0m",
                      robot_ns_.c_str(), los_stop_countdown_);
          }
          // Immediately publish stop this tick
          std_msgs::Bool emergency_stop;
          emergency_stop.data = true;
          emergency_stop_pub_.publish(emergency_stop);
          state_machine_ = EMERGENCY_STOP;
          los_stop_countdown_--;
          los_stop_active_ = true;
        }
        return;
      }

      // -------------------- (A3) If we don't even have pose/goal, do nothing (but latch already handled above) --------------------
      if(!have_geometry_local || !have_goal_local) {
        // if previously stopped, release once (optional)
        if (los_stop_active_) {
          std_msgs::Bool emergency_stop;
          emergency_stop.data = false;
          emergency_stop_pub_.publish(emergency_stop);
          los_stop_active_ = false;
          ROS_WARN_THROTTLE(0.5, "[STOP RELEASE due to missing geometry/goal] %s", robot_ns_.c_str());
        }
        return;
      }

      // -------------------- (B) COLLECT REQUIRED DATA (with diagnostics instead of silent return) --------------------
      bool other_ready = true;
      bool neighbors_ready = true;

      {
        std::lock_guard<std::mutex> lock(mtx_);

        // other robots poses must be ready
        for (int id = 1; id <= num_robots_; ++id) {
          if (id == robot_index_) continue;
          if (id < 1 || id >= static_cast<int>(has_other_pose_by_id_.size()) || !has_other_pose_by_id_[id]) {
            other_ready = false;
            ROS_WARN_THROTTLE(1.0, "[WAIT] %s missing other pose id=%d", robot_ns_.c_str(), id);
            break;
          }
        }

        // neighbors pose + hull must be ready (based on current neighbors_)
        for (int nb : neighbors_) {
          if (nb < 1 || nb > num_robots_) { neighbors_ready = false; break; }
          if (nb >= static_cast<int>(has_connected_robots_pose_by_id_.size()) ||
              nb >= static_cast<int>(has_connected_convexhulls_by_id_.size()) ||
              !has_connected_robots_pose_by_id_[nb] ||
              !has_connected_convexhulls_by_id_[nb]) {
            neighbors_ready = false;
            ROS_WARN_THROTTLE(1.0, "[WAIT] %s missing connected pose/hull nb=%d", robot_ns_.c_str(), nb);
            break;
          }
        }

        // If not ready, we cannot compute LoS; we still proceed to collision stop + planning gating.
        // But we will NOT silently return without logs anymore.
        if (other_ready && neighbors_ready) {
          neighbors_local = neighbors_;

          connected_robots_poses_.reserve(neighbors_local.size());
          connected_robots_convexhull_.reserve(neighbors_local.size());
          for (int id : neighbors_local) {
            connected_robots_poses_.push_back(connected_robots_pose_by_id_[id]);
            connected_robots_convexhull_.push_back(connected_robots_convexhull_by_id_[id]);
          }

          other_robots_poses_.reserve(std::max(0, num_robots_ - 1));
          for (int id = 1; id <= num_robots_; ++id) {
            if (id == robot_index_) continue;
            other_robots_poses_.push_back(other_robots_pose_by_id_[id]);
          }
        }
      }

      // -------------------- (C) HARD COLLISION STOP (kept) --------------------
      if (sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_local.x(),
                                                  current_state_XYTheta_local.y())) < 0.0) {
        std_msgs::Bool emergency_stop;
        emergency_stop.data = true;
        emergency_stop_pub_.publish(emergency_stop);

        {
          std::lock_guard<std::mutex> lock(mtx_);
          state_machine_ = EMERGENCY_STOP;
        }

        ROS_ERROR("EMERGENCY_STOP!!! %s Too close to obstacle!!! dist=%.3f",
                  robot_ns_.c_str(),
                  sdfmap_->getDistanceReal(Eigen::Vector2d(current_state_XYTheta_local.x(),
                                                          current_state_XYTheta_local.y())));
        return;
      }

      // -------------------- (D) LoS STOP (only when neighbors data ready) --------------------
      // If data not ready, we skip LoS check this tick (but latch + graph_emg already handled).
      std::vector<Eigen::Vector3d> scp_info;
      double min_los = std::numeric_limits<double>::infinity();
      bool los_computed = false;
      bool too_far_from_neighbors = false;
      if (neighbors_ready) {
        scp_info.resize(connected_robots_poses_.size());
        for (int i = 0; i < static_cast<int>(connected_robots_poses_.size()); i++) {
          scp_info[i] = scp_->starConvexConstraint(
            {current_state_XYTheta_local.x(), current_state_XYTheta_local.y()},
            connected_robots_poses_[i],
            connected_robots_convexhull_[i],
            false);
            if (hypot(current_state_XYTheta_local.x() - connected_robots_poses_[i][0], current_state_XYTheta_local.y() - connected_robots_poses_[i][1]) > 2.5) {
              too_far_from_neighbors = true;
              break;
            }
        }

        for (int i = 0; i < static_cast<int>(scp_info.size()); ++i) {
          const double temp_los = -scp_info[i][0];
          if (temp_los < min_los) min_los = temp_los;
        }
        los_computed = true;

        const bool los_violate = ((min_los + los_bias_) <= los_margin_);

        if (los_violate && target_type_local) {
          {
            std::lock_guard<std::mutex> lock(mtx_);
            if (los_stop_countdown_ == 0) {
              los_stop_countdown_ = std::max(1, los_stop_rounds_);
              ROS_ERROR("\033[41;37m[LoS STOP TRIGGER] %s min_los=%.3f(+%.2f=%.3f) th=%.3f rounds=%d\033[0m",
                        robot_ns_.c_str(), min_los, los_bias_, min_los + los_bias_, los_margin_, los_stop_countdown_);
            }

            std_msgs::Bool emergency_stop;
            emergency_stop.data = true;
            emergency_stop_pub_.publish(emergency_stop);
            state_machine_ = EMERGENCY_STOP;

            los_stop_countdown_--;
            los_stop_active_ = true;
          }
          return;
        }
      } else {
        // Data not ready: don't compute LoS; optional conservative action:
        // If you want "more conservative", uncomment next 3 lines:
        // std_msgs::Bool emergency_stop; emergency_stop.data = true;
        // emergency_stop_pub_.publish(emergency_stop);
        // return;
      }

      // -------------------- (E) Release emergency_stop if previously active (publish false once) --------------------
      if (los_stop_active_) {
        std_msgs::Bool emergency_stop;
        emergency_stop.data = false;
        emergency_stop_pub_.publish(emergency_stop);
        los_stop_active_ = false;

        ROS_WARN_THROTTLE(0.5,
          "[STOP RELEASE] %s (los_computed=%d min_los=%.3f(+%.2f=%.3f) th=%.3f)",
          robot_ns_.c_str(),
          (int)los_computed, min_los, los_bias_, min_los + los_bias_, los_margin_);
      }

      // -------------------- (F) PLANNING / REPLANNING (same structure) --------------------
      if(state_machine_local == StateMachine::IDLE ||
        ((state_machine_local == StateMachine::PLANNING || state_machine_local == StateMachine::REPLAN) &&
          (ros::Time::now() - loop_start_time_).toSec() > replan_time_)) {

        loop_start_time_ = ros::Time::now();
        double current = loop_start_time_.toSec();

        // start new plan
        if(state_machine_local == StateMachine::IDLE){
          {
            std::lock_guard<std::mutex> lock(mtx_);
            state_machine_ = StateMachine::PLANNING;
          }
          plan_start_time_ = -1;
          predicted_traj_start_time_ = -1;
          plan_start_state_XYTheta = current_state_XYTheta_local;
          plan_start_state_VAJ     = current_state_VAJ_local;
          plan_start_state_OAJ     = current_state_OAJ_local;
        }
        else if(state_machine_local == StateMachine::PLANNING || state_machine_local == StateMachine::REPLAN){

          if(((current_state_XYTheta_local - goal_state_local).head(2).squaredNorm()
              + fmod(fabs((plan_start_state_XYTheta - goal_state_local)[2]), 2.0 * M_PI) * 0.02 < 1.0) ||
            msplanner_->final_traj_.getTotalDuration() < max_replan_time_){
            std::lock_guard<std::mutex> lock(mtx_);
            state_machine_ = StateMachine::GOINGTOGOAL;
            return;
          }

          {
            std::lock_guard<std::mutex> lock(mtx_);
            state_machine_ = StateMachine::REPLAN;
          }

          predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
          msplanner_->get_the_predicted_state(predicted_traj_start_time_,
                                              plan_start_state_XYTheta,
                                              plan_start_state_VAJ,
                                              plan_start_state_OAJ);
        }

        // front end
        ros::Time astar_start_time = ros::Time::now();
        if(!findJPSRoad()){
          {
            std::lock_guard<std::mutex> lock(mtx_);
            state_machine_ = EMERGENCY_STOP;
          }
          ROS_ERROR("EMERGENCY_STOP!!! %s can not find astar road !!!", robot_ns_.c_str());
          std_msgs::Bool emergency_stop;
          emergency_stop.data = true;
          emergency_stop_pub_.publish(emergency_stop);
          return;
        }
        ROS_INFO("\033[41;37m all of front end time:%f \033[0m", (ros::Time::now() - astar_start_time).toSec());

        // -------------------- scale_down (ONLY for target_type_local) --------------------
        double scale_down_param = 1.0;

        // 只有当 target_type_local==true 才进行缩放；否则(ORANGE)保持 1.0
        if (target_type_local) {
          if (too_far_from_neighbors) {
            ROS_ERROR("TOO FAR FROM NEIGHBORS!!! EMERGENCY STOP!");
            scale_down_param = 0.25;
          }
          else if (los_computed) {
            // 你原来的门限：min_los + bias <= 6.0 才开始缩放，否则 1.0
            // if (min_los + los_bias_ <= 5.0) {
              scale_down_param = 1.0 - std::exp(-alpha_ * min_los);
              scale_down_param = std::max(sd_threshold_, scale_down_param);
              // scale_down_param = 0.5;
              // 你原来的极端危险强制更小（只对 GREEN 生效）
              if (min_los + los_bias_ <= 1.0) {
                scale_down_param = 0.05;
              }
            // } else {
            //   scale_down_param = 1.0;
            // }
          } 
          else {
            // GREEN 但 LoS 不可用：别乱缩放
            scale_down_param = 1.0;
          }
        } else {
          // ORANGE: 强制不缩放
          scale_down_param = 1.0;
        }

        std_msgs::Float64 msg;
        msg.data = scale_down_param;
        scale_down_param_pub_.publish(msg);
        msplanner_->set_scale_down_param(scale_down_param);

        ROS_INFO_THROTTLE(0.2,
          "[scale_down] %s target_type=%d los_ok=%d min_los=%.3f bias=%.2f margin=%.2f -> scale=%.3f",
          robot_ns_.c_str(),
          (int)target_type_local,
          (int)los_computed,
          min_los,
          los_bias_,
          los_margin_,
          scale_down_param);

        if (other_ready) {
          msplanner_->set_other_robots_states(other_robots_poses_);
        }
        if (neighbors_ready) {
          msplanner_->set_connected_robots_states(connected_robots_poses_);
          msplanner_->set_connected_robots_convexhulls(connected_robots_convexhull_);
          msplanner_->set_connected_robots_scps(scp_info);
        }

        bool result = msplanner_->minco_plan(jps_planner_->flat_traj_);
        std_msgs::Bool solving_failed;

        // if(!result && target_type_local){
        //   // {
        //   //   std::lock_guard<std::mutex> lock(mtx_);
        //   //   state_machine_ = EMERGENCY_STOP;
        //   // }
        //   ROS_ERROR("EMERGENCY_STOP! %s Fail to find a feasible solution!!!", robot_ns_.c_str());
        //   std_msgs::Bool emergency_stop;
        //   emergency_stop.data = true;
        //   emergency_stop_pub_.publish(emergency_stop);
        //   return;
        // } 
        // else {
        //   if (target_type_local) {
        //     solving_failed.data = false;
        //     solving_failed_pub_.publish(solving_failed);
        //   }
        // }

        ROS_INFO("Scale down param: %f", scale_down_param);
        ROS_INFO("\033[43;32m all of plan time:%f \033[0m", (ros::Time::now().toSec() - current));

        // visualization
        msplanner_->mincoPathPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPathPath);
        msplanner_->mincoPointPub(msplanner_->final_traj_, plan_start_state_XYTheta, visualizer_->mincoPointMarker, Eigen::Vector3d(239, 41, 41));

        // for replan
        if(plan_start_time_ < 0){
          Traj_start_time_ = ros::Time::now();
          plan_start_time_ = Traj_start_time_.toSec();
        } else {
          plan_start_time_ = current + max_replan_time_;
          Traj_start_time_ = ros::Time(plan_start_time_);
        }

        MPCPathPub(plan_start_time_);
        Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
      }

      // -------------------- (G) FINISH CHECK --------------------
      if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_){
        std::lock_guard<std::mutex> lock(mtx_);
        state_machine_ = StateMachine::IDLE;
        have_goal_ = false;
      }
    }
    // ======================= END REPLACE =======================

    // ======= unchanged below (except formatting) =======

    bool findJPSRoad(){
      jps_planner_->set_scale_down_param(msplanner_->get_scale_down_param());
      ros::Time current = ros::Time::now();
      Eigen::Vector3d start_state;
      std::vector<Eigen::Vector3d> start_path;
      std::vector<Eigen::Vector3d> start_path_both_end;
      bool if_forward = true;

      if(plan_start_time_ > 0){
        start_path = msplanner_->get_the_predicted_state_and_path(
          predicted_traj_start_time_,
          predicted_traj_start_time_ + jps_planner_->jps_truncation_time_,
          plan_start_state_XYTheta,
          start_state,
          if_forward);

        u_int start_path_size = start_path.size();
        u_int start_path_i = 0;
        for(; start_path_i < start_path_size; start_path_i++){
          if(!jps_planner_->JPS_check_if_collision(start_path[start_path_i].head(2)))
            break;
        }

        if (start_path.empty()) {
          start_state = plan_start_state_XYTheta;
        }
        else if(start_path_i == 0){
          start_state = plan_start_state_XYTheta;
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        else if(start_path_i < start_path_size){
          start_path = std::vector<Eigen::Vector3d>(start_path.begin(), start_path.begin() + start_path_i);
          start_state = start_path.back();
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
        else{
          start_path_both_end.push_back(start_path.front());
          start_path_both_end.push_back(start_state);
        }
      }
      else{
        start_state = plan_start_state_XYTheta;
      }

      jps_planner_->plan(start_state, goal_state_);
      jps_planner_->getKinoNodeWithStartPath(start_path, if_forward, plan_start_state_VAJ, plan_start_state_OAJ);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "jps_planner";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 11;
      marker.pose.position.y = 8;
      marker.pose.position.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.5;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      double search_time = (ros::Time::now() - current).toSec() * 1000.0;
      std::ostringstream out;
      out << std::fixed << "JPS: \n" << std::setprecision(2) << search_time << " ms";
      marker.text = out.str();
      record_pub_.publish(marker);

      ROS_INFO("\033[40;36m jps_planner_ search time:%lf  \033[0m", (ros::Time::now() - current).toSec());
      return true;
    }

    void MPCPathPub(const double& traj_start_time){
      (void)traj_start_time;

      Eigen::MatrixXd initstate = msplanner_->get_current_iniState();
      Eigen::MatrixXd finState = msplanner_->get_current_finState();
      Eigen::MatrixXd finalInnerpoints = msplanner_->get_current_Innerpoints();
      Eigen::VectorXd finalpieceTime = msplanner_->get_current_finalpieceTime();
      Eigen::Vector3d iniStateXYTheta = msplanner_->get_current_iniStateXYTheta();

      carstatemsgs::Polynome polynome;
      polynome.header.frame_id = "map";
      polynome.header.stamp = ros::Time::now();
      polynome.init_p.x = initstate.col(0).x();
      polynome.init_p.y = initstate.col(0).y();
      polynome.init_v.x = initstate.col(1).x();
      polynome.init_v.y = initstate.col(1).y();
      polynome.init_a.x = initstate.col(2).x();
      polynome.init_a.y = initstate.col(2).y();
      polynome.tail_p.x = finState.col(0).x();
      polynome.tail_p.y = finState.col(0).y();
      polynome.tail_v.x = finState.col(1).x();
      polynome.tail_v.y = finState.col(1).y();
      polynome.tail_a.x = finState.col(2).x();
      polynome.tail_a.y = finState.col(2).y();

      if(plan_start_time_ < 0) polynome.traj_start_time = ros::Time::now();
      else polynome.traj_start_time = ros::Time(plan_start_time_);

      for(u_int i = 0; i < finalInnerpoints.cols(); i++){
        geometry_msgs::Vector3 point;
        point.x = finalInnerpoints.col(i).x();
        point.y = finalInnerpoints.col(i).y();
        point.z = 0.0;
        polynome.innerpoints.push_back(point);
      }
      for(u_int i = 0; i < finalpieceTime.size(); i++){
        polynome.t_pts.push_back(finalpieceTime[i]);
      }
      polynome.start_position.x = iniStateXYTheta.x();
      polynome.start_position.y = iniStateXYTheta.y();
      polynome.start_position.z = iniStateXYTheta.z();

      if(!msplanner_->if_standard_diff_){
        polynome.ICR.x = msplanner_->ICR_.x();
        polynome.ICR.y = msplanner_->ICR_.y();
        polynome.ICR.z = msplanner_->ICR_.z();
      }

      mpc_polynome_pub_.publish(polynome);
    }
};

#endif