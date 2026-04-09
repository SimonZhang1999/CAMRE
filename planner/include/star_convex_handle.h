#ifndef SRC_STAR_CONVEX_HANDLE_H
#define SRC_STAR_CONVEX_HANDLE_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace star_convex_handle {
    using Vec2 = Eigen::Vector2d;
    class StarConvexHandler
    {
    public:
        StarConvexHandler(ros::NodeHandle& nh)
        {
            sub_ = nh.subscribe("robot_1/vertices_convexhull", 10,
                                &StarConvexHandler::starConvexConstraint, this);
            ros::param::get("/flip_radius", flipRadius);
            ros::param::get("/alpha_relax", alpha);
            ros::param::get("/visiable_range", visiableRange);
            // ROS_INFO("SCP radius: %f, visiable range: %f, alpha: %f", flipRadius, visiableRange, alpha);

            ros::param::get("/d_los_max", flipD);
            ros::param::get("/d_los_min", flipD_min);
            ros::param::get("/k_omega", kS);
            ros::param::get("/m_d_min", m_d_min);
            ros::param::get("/m_lambda", m_lambda);
        }

        const std::vector<Eigen::Vector2d>& getVertices() const {
            return vertices_;
        }
        Eigen::Vector3d starConvexConstraint(const Eigen::Vector2d& x,
                                                            const Eigen::Vector2d& center,
                                                            const std::vector<Eigen::Vector2d>& ch_vertices,
                                                            const bool& is_constraint);
        // double starConvexConstraint(const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices, const bool& is_constraint);
        Eigen::Vector2d gradOfSCPConstraint(const double& J_constraint, const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices);
        Eigen::Matrix2d ComputeSCPx(const Eigen::Vector2d& x);
        double sensitivityCost(const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices);
        Eigen::Vector2d pointTransform(const Eigen::Vector2d& x);
        Eigen::Vector2d getOuterNormalFactor(const std::vector<Eigen::Vector2d>& plane, const Eigen::Vector2d& center);
        double getPoint2PlaneDistance(const Eigen::Vector2d& point, 
                                      const std::vector<Eigen::Vector2d>& plane, 
                                      const Eigen::Vector2d& center);
        double getThetak(const Eigen::Vector2d& dir,
                                            const Eigen::Vector2d& point, 
                                            const std::vector<Eigen::Vector2d>& plane, 
                                            const Eigen::Vector2d& center);
        double logSumExpCap(const Eigen::Array<long double, -1, 1>& distances);
        std::vector<Eigen::Vector2d> getNormalvectors(const std::vector<Eigen::Vector2d>& ch_vertices, const Eigen::Vector2d& center);

        double penalty(double x);
        // void publishPolygonMarker(
        //         ros::Publisher &pub,
        //         const std::vector<Eigen::Vector2d> &vertices
        // );
        void drawPoints(
            ros::Publisher& pub,
            const std::vector<geometry_msgs::Point>& points
        );
        double supportFunction(const std::vector<Vec2>& poly, const Vec2& d);
        std::vector<Vec2> approximateOuterHull(const std::vector<Vec2>& original_poly,
                                                                  int num_directions);   
    private:
        void starConvexConstraint(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
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
            vertices_ = std::move(tmp);
        }

        ros::Subscriber sub_;
        std::vector<Eigen::Vector2d> vertices_;
        float visiableRange; // Used to add augmented points into raw pointCloud
        float flipRadius;    // Radius for spherical flipping
        float testPointX, testPointY;
        float alpha; // Coefficient in log-exp-sum relaxation

        float flipD;     // Safe distance (to convexHull) to ignore visible constraints
        float flipD_min; // Mimimum allowed distance to convexHull to maintain visible
        float kS;        // Constant of the weight function
        float m_d_min;
        float m_lambda;
    };
}

#endif // SRC_STAR_CONVEX_HANDLE_H