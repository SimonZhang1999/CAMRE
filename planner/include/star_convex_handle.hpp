#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>

namespace star_convex {
    class StarConvexHandler
    {
    public:
        StarConvexHandler(ros::NodeHandle& nh)
        {
            sub_ = nh.subscribe("vertices_convexhull", 10,
                                &StarConvexHandler::starConvexConstraint, this);
        }

        const std::vector<Eigen::Vector2d>& getVertices() const {
            return vertices_;
        }
        double starConvexConstraint(const Eigen::Vector2d& x, const Eigen::Vector2d& center, std::vector<Eigen::Vector2d> ch_vertices);
        double sensitivityCost(const Eigen::Vector2d& x, const Eigen::Vector2d& center, std::vector<Eigen::Vector2d> ch_vertices);
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
    };
}