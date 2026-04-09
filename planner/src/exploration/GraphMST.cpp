#include "GraphMST.hpp"
#include "star_convex_handle.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <limits>

class MSTFromPosesNode {
public:
    explicit MSTFromPosesNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh) {
        chh = std::make_shared<star_convex_handle::StarConvexHandler>(nh_);
        // Parameters
        pnh.param("num_robots", num_robots_, 0);
        pnh.param("rate_hz", rate_hz_, 10.0);
        pnh.param("max_edge_distance", max_edge_distance_, -1.0); // <=0 means no gating

        // marker params (optional)
        pnh.param<std::string>("global_frame", global_frame_, std::string("map"));
        pnh.param("edge_line_width", edge_line_width_, 0.05);
        pnh.param("marker_lifetime", marker_lifetime_, 0.2);
        pnh.param("use_gradient", use_gradient_, true);

        if (num_robots_ <= 0) {
            ROS_FATAL("Parameter '~num_robots' must be > 0");
            throw std::runtime_error("invalid num_robots");
        }

        poses_.resize(num_robots_);
        has_pose_.assign(num_robots_, false);

        vertices_.resize(num_robots_);
        has_vertices_convexhull_.assign(num_robots_, false);

        // Subscribers
        subs_pose_.resize(num_robots_);
        for (int i = 0; i < num_robots_; ++i) {
            subs_pose_[i] = nh_.subscribe<nav_msgs::Odometry>(
                "/robot_"+std::to_string(i+1)+"/RosAria/odom", 1,
                boost::bind(&MSTFromPosesNode::poseCb, this, _1, i)
            );
        }

        subs_vertices_convexhull_.resize(num_robots_);
        for (int i = 0; i < num_robots_; ++i) {
            subs_vertices_convexhull_[i] = nh_.subscribe<std_msgs::Float32MultiArray>(
                "/robot_"+std::to_string(i+1)+"/vertices_convexhull", 1,
                boost::bind(&MSTFromPosesNode::verticex_convexhullCb, this, _1, i)
            );
        }
    
        // Publisher for MST edges visualization
        pub_mst_edges_ = nh_.advertise<visualization_msgs::Marker>("mst_edges_marker", 1);

        // Publish the graph and weights
        pub_graph_edges_ = nh_.advertise<std_msgs::Float32MultiArray>("mst_graph/edges", 1);
        pub_graph_los_ = nh.advertise<std_msgs::Float64MultiArray>("/record_graph_edges", 10);
        // Timer loop
        timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_),
                                 &MSTFromPosesNode::timerCb, this);

        ROS_INFO("MSTFromPosesNode started: N=%d, rate=%.2f Hz", num_robots_, rate_hz_);
    }

private:
    std::shared_ptr<star_convex_handle::StarConvexHandler> chh;
    void poseCb(const nav_msgs::Odometry::ConstPtr& msg, int idx) {
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

    void verticex_convexhullCb(const std_msgs::Float32MultiArray::ConstPtr& msg, int idx)
    {
        std::lock_guard<std::mutex> lock(mtx_);
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
            tmp.emplace_back(static_cast<double>(data[i]),
                            static_cast<double>(data[i+1]));
        }
        vertices_[idx] = std::move(tmp);
        has_vertices_convexhull_[idx] = true;
    }

    static double dist2D(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
        const double dx = a.position.x - b.position.x;
        const double dy = a.position.y - b.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // C++ version of your "weight_to_rgb" style mapping.
    // Here I make it truly reach blue at w=1.
    static std::array<float, 3> weightToRGB(float w) {
        w = std::clamp(w, 0.0f, 1.0f);

        float r, g, b;
        if (w < 0.5f) {
            // Red (1,0,0) -> Yellow (1,1,0)
            r = 1.0f;
            g = 2.0f * w;
            b = 0.0f;
        } else {
            // Yellow (1,1,0) -> Blue (0,0,1)
            float t = 2.0f * (w - 0.5f); // 0..1
            r = 1.0f - t;                // 1..0
            g = 1.0f - t;                // 1..0
            b = t;                       // 0..1
        }
        return {r, g, b};
    }

    bool allPosesReady() const {
        for (bool ok : has_pose_) if (!ok) return false;
        return true;
    }

    bool allVerticesConvexhullReady() const {
        for (bool ok : has_vertices_convexhull_) if (!ok) return false;
        return true;
    }    

    void publishMSTMarker(const std::vector<geometry_msgs::Pose>& poses_local,
                        const std::vector<std::vector<double>>& selected)
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = global_frame_;
        mk.header.stamp = ros::Time::now();
        mk.ns = "mst_edges";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::LINE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = edge_line_width_;
        mk.lifetime = ros::Duration(marker_lifetime_);

        // 如果没有边，直接发布一个空 marker，用于清除旧显示
        if (selected.empty()) {
            pub_mst_edges_.publish(mk);
            return;
        }

        // ------------------------------------------------------------
        // 1) 计算归一化尺度 d_norm
        //    优先使用 max_edge_distance_
        //    若未设置，则使用当前 selected 中最大的权重 e[2]
        // ------------------------------------------------------------
        double d_norm = max_edge_distance_;
        if (d_norm <= 0.0) {
            d_norm = 1e-6;
            for (const auto& e : selected) {
                if (e.size() < 3) continue;
                const double d = e[2];   // 使用真实边权
                if (std::isfinite(d) && d > d_norm) {
                    d_norm = d;
                }
            }
        }

        mk.points.reserve(selected.size() * 2);
        mk.colors.reserve(selected.size() * 2);

        // ------------------------------------------------------------
        // 2) 遍历每条边
        //    e = {i, j, d}
        // ------------------------------------------------------------
        for (const auto& e : selected) {
            if (e.size() < 3) continue;

            const int i = static_cast<int>(e[0]);
            const int j = static_cast<int>(e[1]);
            const double d = e[2];   // 真实权重

            // 安全检查
            if (i < 0 || i >= static_cast<int>(poses_local.size()) ||
                j < 0 || j >= static_cast<int>(poses_local.size()) ||
                !std::isfinite(d)) {
                continue;
            }

            geometry_msgs::Point pi, pj;
            pi.x = poses_local[i].position.x;
            pi.y = poses_local[i].position.y;
            pi.z = 0.05;

            pj.x = poses_local[j].position.x;
            pj.y = poses_local[j].position.y;
            pj.z = 0.05;

            // --------------------------------------------------------
            // 3) 归一化权重
            //    norm_d in [0,1]
            //    如果 d 越大代表权重越高，则直接用 norm_d
            // --------------------------------------------------------
            float norm_d = static_cast<float>(std::clamp(d / d_norm, 0.0, 1.0));

            // --------------------------------------------------------
            // 4) 映射为灰度
            //    高权重 -> 黑色
            //    低权重 -> 浅灰
            //
            //    gray=0   表示黑
            //    gray=1   表示白
            //
            //    这里让：
            //      norm_d = 1.0 -> gray = 0.0   (纯黑)
            //      norm_d = 0.0 -> gray = 0.8   (浅灰)
            // --------------------------------------------------------
            float gray1 = 0.8f * (1.0f - norm_d);

            std_msgs::ColorRGBA c1;
            c1.r = gray1;
            c1.g = gray1;
            c1.b = gray1;
            c1.a = 1.0f;

            std_msgs::ColorRGBA c2 = c1;

            // --------------------------------------------------------
            // 5) 可选渐变效果
            //    起点颜色 c1 稍深
            //    终点颜色 c2 稍浅
            // --------------------------------------------------------
            if (use_gradient_) {
                float gray2 = std::min(gray1 + 0.20f, 0.90f);
                c2.r = gray2;
                c2.g = gray2;
                c2.b = gray2;
                c2.a = 1.0f;
            }

            mk.points.push_back(pi);
            mk.points.push_back(pj);

            // LINE_LIST 要求 colors.size() 与 points.size() 一致
            mk.colors.push_back(c1);
            mk.colors.push_back(use_gradient_ ? c2 : c1);
        }

        pub_mst_edges_.publish(mk);
    }

    double CalculateWeights(std::vector<geometry_msgs::Pose> poses_local, 
                            std::vector<std::vector<Eigen::Vector2d>>& vertices, const int& robot_i, const int& robot_j, double& los_dmin) {
        Eigen::Vector2d center = {poses_local[robot_j].position.x, poses_local[robot_j].position.y};
        Eigen::Vector2d x = {poses_local[robot_i].position.x, poses_local[robot_i].position.y};
        auto visble_info = chh->starConvexConstraint(x, center, vertices[robot_j], false);
        // double sensitivity_cost = chh.sensitivityCost(x, center, vertices[robot_j]);
        // ROS_INFO("Visible cost: %f, Sensitivity cost:%f", visble_cost, sensitivity_cost);
        los_dmin =  visble_info[0];
        if (visble_info[0] > 0) return std::numeric_limits<double>::infinity(); 
        double d = dist2D(poses_local[robot_i], poses_local[robot_j]);
        if (max_edge_distance_ > 0.0 && d > max_edge_distance_) {
            return d = std::numeric_limits<double>::infinity(); 
        }
        return d;
    } 

    void timerCb(const ros::TimerEvent&) {
        auto t_total_start = std::chrono::steady_clock::now();

        std_msgs::Float64MultiArray record_graph_msg;
        std::vector<geometry_msgs::Pose> poses_local;
        std::vector<std::vector<Eigen::Vector2d>> vertices_local;
        std::vector<std::vector<double>> selected;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!allPosesReady()) {
                static ros::Time last_warn(0);
                if ((ros::Time::now() - last_warn).toSec() > 2.0) {
                    ROS_WARN("Waiting for all poses... (%zu/%d received)",
                            countReceivedUnlocked(), num_robots_);
                    last_warn = ros::Time::now();
                }
                return;
            }
            if (!allVerticesConvexhullReady()) {
                static ros::Time last_warn(0);
                if ((ros::Time::now() - last_warn).toSec() > 2.0) {
                    ROS_WARN("Waiting for all vertices convexulls... (%zu/%d received)",
                            countReceivedUnlocked(), num_robots_);
                    last_warn = ros::Time::now();
                }
                return;
            }
            poses_local = poses_;
            vertices_local = vertices_;
        }

        GraphMST g(num_robots_);
        for (int i = 0; i < num_robots_; ++i) {
            for (int j = i + 1; j < num_robots_; ++j) {
                double los_min;
                auto d = CalculateWeights(poses_local, vertices_local, i, j, los_min);
                if (d != std::numeric_limits<double>::infinity()) {
                    g.addEdge(i, j, d);
                    record_graph_msg.data.push_back(i);
                    record_graph_msg.data.push_back(j);
                    record_graph_msg.data.push_back(los_min);
                }
            }
        }

        const bool connected = g.is_graph_connected();
        g.KruskalMST(selected);
        pub_graph_los_.publish(record_graph_msg);

        publishMSTMarker(poses_local, selected);
        publishGraph(selected);

        std::string s;
        s += connected ? "[CONNECTED] " : "[NOT CONNECTED] ";
        s += "Selected edges: ";
        for (std::size_t k = 0; k < selected.size(); ++k) {
            s += "(" + std::to_string(selected[k][0]) + "," + std::to_string(selected[k][1]) + ")";
            if (k + 1 < selected.size()) s += ", ";
        }
        ROS_INFO_STREAM_THROTTLE(1.0, s);

        auto t_total_end = std::chrono::steady_clock::now();
        double total_ms =
            std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

        ROS_INFO_STREAM_THROTTLE(1.0, "timerCb total time = " << total_ms << " ms");
    }

    void publishGraph(const std::vector<std::vector<double>>& selected) {
        // edges: [u0,v0,u1,v1,...]
        std_msgs::Float32MultiArray edges_msg;
        edges_msg.data.reserve(selected.size() * 3);

        for (const auto& e : selected) {
            double u = e[0];
            double v = e[1];
            double w = e[2];
            edges_msg.data.push_back(u+1);
            edges_msg.data.push_back(v+1);
            edges_msg.data.push_back(w);
        }
        pub_graph_edges_.publish(edges_msg);
    }


    std::size_t countReceivedUnlocked() const {
        std::size_t c = 0;
        for (bool ok : has_pose_) if (ok) ++c;
        return c;
    }

private:
    ros::NodeHandle nh_;
    int num_robots_{0};
    double rate_hz_{10.0};
    double max_edge_distance_{-1.0};

    // marker configs
    std::string global_frame_{"map"};
    double edge_line_width_{0.05};
    double marker_lifetime_{0.2};
    bool use_gradient_{true};

    std::vector<ros::Subscriber> subs_pose_;
    std::vector<ros::Subscriber> subs_vertices_convexhull_;

    ros::Publisher pub_mst_edges_;
    ros::Publisher pub_graph_edges_;
    ros::Publisher pub_graph_los_;

    mutable std::mutex mtx_;
    std::vector<geometry_msgs::Pose> poses_;
    std::vector<std::vector<Eigen::Vector2d>> vertices_;
    std::vector<bool> has_pose_;
    std::vector<bool> has_vertices_convexhull_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mst_from_poses");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        MSTFromPosesNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to start node: %s", e.what());
        return 1;
    }
    return 0;
}