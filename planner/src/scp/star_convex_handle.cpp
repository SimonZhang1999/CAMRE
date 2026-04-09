#include "star_convex_handle.hpp"

float visiableRange; // Used to add augmented points into raw pointCloud
float flipRadius;    // Radius for spherical flipping
float testPointX, testPointY;
float alpha; // Coefficient in log-exp-sum relaxation

float flipD;     // Safe distance (to convexHull) to ignore visible constraints
float flipD_min; // Mimimum allowed distance to convexHull to maintain visible
float kS;        // Constant of the weight function
float m_d_min;
float m_lambda;

namespace star_convex {
    /* Function pointTransform
    operates point transformation that flips the point x to outside of the
    sphere boundary, whose radius is m_r_ball. */
    Eigen::Vector2d StarConvexHandler::pointTransform(const Eigen::Vector2d& x)
    {

        return x + 2 * (flipRadius - x.norm()) * x.normalized();

    }

    /* Function getOuterNormalFactor
    calculates the outer normal factor of a plane using 3 points, 
    which are sorted counterclockwise. */
    Eigen::Vector2d StarConvexHandler::getOuterNormalFactor(const std::vector<Eigen::Vector2d>& plane, const Eigen::Vector2d& center)
    {
        // plane: 至少 2 个点（这里你原来用 3 个点，可以保留 3 个，也可以只用 0 和 1）
        const Eigen::Vector2d& p0 = plane[0];
        const Eigen::Vector2d& p1 = plane[1];

        // 边向量
        Eigen::Vector2d edge = p0 - p1;

        // 左法向量（逆时针旋转 90°）
        Eigen::Vector2d normal(-edge.y(), edge.x());
        normal.normalize();

        // 判断是否为外法线: 如果 normal 指向 center，就反向
        // if (normal.dot(p0 - center) < 0)
        // {
        //     normal = -normal;
        // }

        return normal;
    }

    /* Function getPoint2PlaneDistance
    calculates the signed distance from a point to a plane. */
    // x_cap
    double StarConvexHandler::getPoint2PlaneDistance(const Eigen::Vector2d& point, 
                                                     const std::vector<Eigen::Vector2d>& plane, 
                                                     const Eigen::Vector2d& center)
    {

        return getOuterNormalFactor(plane, center).dot(point - plane[0]);

    }

    double StarConvexHandler::getThetak(const Eigen::Vector2d& dir,
                                        const Eigen::Vector2d& point, 
                                        const std::vector<Eigen::Vector2d>& plane, 
                                        const Eigen::Vector2d& center)
    {

        return getOuterNormalFactor(plane, center).dot(dir);

    }

    /* Function logSumExpCap
    calculates the difference value between the safe margin and the maximum 
    of distances. The maximum is approximated using log-sum-exp function. */
    double StarConvexHandler::logSumExpCap(const Eigen::Array<long double, -1, 1>& distances)
    {
        double d_max = std::log((alpha * distances).exp().sum()) / alpha;
        // double d_max = distances.matrix().maxCoeff();
        // std::cout << "[debug]: " << ans << " " << row << " " << col << std::endl;

        // double d_max = std::log((m_alpha * distances_normalized).exp().sum()) / m_alpha * distances_norm;
        // // std::cout << d_max << std::endl;
        // while (isinf(d_max) || isnan(d_max)) {
        //     // if (isinf(d_max)) ROS_ERROR("[optimizer]: LSE overflow!");
        //     // if (isnan(d_max)) ROS_ERROR("[optimizer]: LSE illegal operation!");
        //     alpha /= 2.0;
        //     d_max = std::log((alpha * distances).exp().sum()) / alpha;
        //     std::cout << distances.transpose() << std::endl;
        //     ROS_ERROR("[optimizer]: dmax: %f!", d_max);
        // }
        if (isinf(d_max)) ROS_ERROR("[optimizer]: LSE overflow!");
        if (isnan(d_max)) ROS_ERROR("[optimizer]: LSE illegal operation!");
        return m_d_min - d_max;

    }

    std::vector<Eigen::Vector2d> StarConvexHandler::getNormalvectors(const std::vector<Eigen::Vector2d>& ch_vertices, const Eigen::Vector2d& center) {
        std::vector<Eigen::Vector2d> normal_vectors;
        int n = ch_vertices.size();
        for (int i = 0; i < n; i++)
        {
            std::vector<Eigen::Vector2d> plane = {ch_vertices[i], ch_vertices[(i+1)%n]};
            normal_vectors.push_back(getOuterNormalFactor(plane, center));
        }       
        return normal_vectors; 
    }

    /* Function starConvexConstraint
    calculates the star convex constraint of a variable in DISTRIBUTED optimization. */
    double StarConvexHandler::starConvexConstraint(const Eigen::Vector2d& x, const Eigen::Vector2d& center, std::vector<Eigen::Vector2d> ch_vertices)
    {
        // Operate point transformation
        Eigen::Vector2d x_cap = pointTransform(x - center);
        // x_cap += m_polytopes[index].center; // 为什么不给人加回去？？因为前面有人负重前行

        // Calculate the distances from x_cap to all polygons
        int n = ch_vertices.size();
        // Eigen::ArrayXd distances(n);
        // distances ∈ R^{n×1} nx1的列向量

        // Eigen::ArrayXd distances(n);
        // distances ∈ R^{n×1} nx1的列向量
        Eigen::Array<long double, -1, 1> distances(n);
        for (int i = 0; i < n; i++)
        {
            distances(i) = getPoint2PlaneDistance(x_cap, {ch_vertices[i], ch_vertices[(i+1)%n]}, center);
        }
        // if (distances(0) != distances(0)) {
        //     std::cout << distances.transpose() << std::endl;
        //     std::cout << x_cap.transpose() << std::endl;
        //     std::cout << x.transpose() << std::endl;
        //     std::cout << m_polytopes[index].center.transpose() << std::endl;
        //     // while (1);
        // }
        double J_constraint = logSumExpCap(distances);

        if (J_constraint > 0)
        {
            return J_constraint;
        }
        else
        {
            return 0;
        }
    }

    double StarConvexHandler::sensitivityCost(const Eigen::Vector2d& x, const Eigen::Vector2d& center, std::vector<Eigen::Vector2d> ch_vertices) {
        // Operate point transformation
        Eigen::Vector2d x_cap = pointTransform(x - center);
        // x_cap += m_polytopes[index].center; // 为什么不给人加回去？？因为前面有人负重前行

        // Calculate the distances from x_cap to all polygons
        int n = ch_vertices.size();
        // Eigen::ArrayXd distances(n);
        // distances ∈ R^{n×1} nx1的列向量
        Eigen::ArrayXd distances(n);
        Eigen::ArrayXd cos_thetas(n);  // n = 0
        Eigen::Vector2d dir = (x - center).normalized();
        for (int i = 0; i < n; i++)
        {
            distances(i) = getPoint2PlaneDistance(x_cap, {ch_vertices[i], ch_vertices[(i+1)%n]}, center);
            cos_thetas(i) = getThetak(dir, x_cap, {ch_vertices[i], ch_vertices[(i+1)%n]}, center);
        }
        long double d_min = distances.minCoeff();
        long double d_max = distances.maxCoeff();
        // if (distances(0) != distances(0)) {
        //     std::cout << distances.transpose() << std::endl;
        //     std::cout << x_cap.transpose() << std::endl;
        //     std::cout << x.transpose() << std::endl;
        //     std::cout << m_polytopes[index].center.transpose() << std::endl;
        //     // while (1);
        // }

        Eigen::ArrayXd weights = (alpha * distances).exp();
        double numerator   = (weights * cos_thetas).sum();
        double denominator = weights.sum();
        double J_cost = numerator / denominator;
        // 找最大值对应的 index
        Eigen::Index idx_max;
        distances.maxCoeff(&idx_max);

        std::cout << "distances max = " << distances(idx_max) 
                  << ", sensitivity max = " << cos_thetas(idx_max)
                  << ", idx_max = " << idx_max 
                  << std::endl;
        return J_cost;        
    }


    /* Function penalty
    turns a hard constraint to a soft constraint. */
    double StarConvexHandler::penalty(double x)
    {
        return m_lambda * std::pow(std::max(x, 0.0), 3);
    }

    // 画一组点（points 是一个 std::vector<geometry_msgs::Point>）
    void StarConvexHandler::drawPoints(
        ros::Publisher& pub,
        const std::vector<geometry_msgs::Point>& points
    ){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "points_ns";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.points = points;    

        pub.publish(marker);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convex_hull_subscriber");
    ros::NodeHandle nh;
    star_convex::StarConvexHandler chh(nh);
    ros::param::get("/flip_radius", flipRadius);
    ros::param::get("/alpha_relax", alpha);
    ros::param::get("/visiable_range", visiableRange);
    ROS_INFO("SCP radius: %f, visiable range: %f", flipRadius, visiableRange);

    ros::param::get("/d_los_max", flipD);
    ros::param::get("/d_los_min", flipD_min);
    ros::param::get("/k_omega", kS);
    ros::param::get("/m_dim", m_d_min);
    ros::param::get("/m_lambda", m_lambda);
    ros::Rate rate(10);
    ros::Publisher pub_pts = nh.advertise<visualization_msgs::Marker>("three_points", 1);
    while (ros::ok())
    {
        auto vertices = chh.getVertices();
        if (vertices.empty()) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        std::vector<geometry_msgs::Point> pts;

        geometry_msgs::Point p;
        p.z = 0.0;
        Eigen::Vector2d x = {4, -9};
        Eigen::Vector2d center = {0, 0};
        p.x = x(0); p.y = x(1); pts.push_back(p);
        auto x_cap = chh.pointTransform(x);
        p.x = x_cap(0); p.y = x_cap(1); pts.push_back(p);
        p.x = x(0); p.y = x(1); pts.push_back(p);       
        chh.starConvexConstraint(x, center, vertices);
        chh.sensitivityCost(x, center, vertices);
        chh.drawPoints(pub_pts, pts);

        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}
