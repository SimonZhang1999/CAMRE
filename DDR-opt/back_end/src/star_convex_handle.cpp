#include "star_convex_handle.h"

using namespace star_convex_handle;
namespace star_convex_handle {
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
        Eigen::Vector2d e = p1 - p0;

        // 左法向量（逆时针旋转 90°）
        Eigen::Vector2d n(-e.y(), e.x());
        double norm = n.norm();
        if (norm < 1e-12) return Eigen::Vector2d::Zero();
        n /= norm;
        // 判断是否为外法线: 如果 normal 指向 center，就反向
        if (n.dot(center - p0) > 0) n = -n;

        return n;
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
        // if (isinf(d_max)) ROS_ERROR("[optimizer]: LSE overflow!");
        if (isnan(d_max)) ROS_ERROR("[optimizer]: LSE illegal operation!");
        return 0.2 - d_max;

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

    // /* Function starConvexConstraint
    // calculates the star convex constraint of a variable in DISTRIBUTED optimization. */
    // double StarConvexHandler::starConvexConstraint(const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices, const bool& is_constraint)
    // {
    //     // Operate point transformation
    //     Eigen::Vector2d x_cap = center + pointTransform(x - center);
    //     // x_cap += m_polytopes[index].center; // 为什么不给人加回去？？因为前面有人负重前行

    //     // Calculate the distances from x_cap to all polygons
    //     int n = ch_vertices.size();
    //     // Eigen::ArrayXd distances(n);
    //     // distances ∈ R^{n×1} nx1的列向量

    //     // Eigen::ArrayXd distances(n);
    //     // distances ∈ R^{n×1} nx1的列向量
    //     Eigen::Array<long double, -1, 1> distances(n);
    //     for (int i = 0; i < n; i++)
    //     {
    //         distances(i) = getPoint2PlaneDistance(x_cap, {ch_vertices[i], ch_vertices[(i+1)%n]}, center);
    //     }
    //     // if (distances(0) != distances(0)) {
    //     //     std::cout << distances.transpose() << std::endl;
    //     //     std::cout << x_cap.transpose() << std::endl;
    //     //     std::cout << x.transpose() << std::endl;
    //     //     std::cout << m_polytopes[index].center.transpose() << std::endl;
    //     //     // while (1);
    //     // }
    //     double J_constraint = logSumExpCap(distances);
    //     if (is_constraint) {
    //         if (J_constraint > 0)
    //         {
    //             return J_constraint;
    //         }
    //         else
    //         {
    //             return 0;
    //         }
    //     }
    //     else return J_constraint;
    // }

    Eigen::Vector3d StarConvexHandler::starConvexConstraint(const Eigen::Vector2d& x,
                                                            const Eigen::Vector2d& center,
                                                            const std::vector<Eigen::Vector2d>& ch_vertices,
                                                            const bool& is_constraint)
    {
        Eigen::Vector3d ret = Eigen::Vector3d::Zero();
        const int n = (int)ch_vertices.size();
        if (n < 2) return ret;

        // -------------------------
        // 1) x_cap
        // -------------------------
        const Eigen::Vector2d xc = x - center;
        const Eigen::Vector2d x_cap = center + pointTransform(xc);

        // -------------------------
        // 2) Precompute per-edge normals + signed distances d_i
        //    N: n×2, d: n×1
        // -------------------------
        Eigen::MatrixXd N(n, 2);
        Eigen::ArrayXd d(n);

        for (int i = 0; i < n; ++i) {
            const Eigen::Vector2d& p0 = ch_vertices[i];
            const Eigen::Vector2d& p1 = ch_vertices[(i + 1) % n];

            // outer normal (unit)
            Eigen::Vector2d n_i = getOuterNormalFactor({p0, p1}, center);
            N.row(i) = n_i.transpose();

            // signed distance
            d(i) = n_i.dot(x_cap - p0);
        }

        // -------------------------
        // 3) LSE max approximation: d_max = log(sum(exp(alpha*d)))/alpha
        //    J = margin - d_max (your margin is 0.2)
        // -------------------------
        // Numerically safer: subtract max before exp
        const double a = alpha;
        const Eigen::ArrayXd w = (a * d).exp(); // shifted
        const double denom = w.sum();

        double d_max = std::log(denom) / a;
        if (std::isnan(d_max)) ROS_ERROR("[optimizer]: LSE illegal operation!");

        const double J_raw = 1.0 - d_max;   // same as your logSumExpCap()

        // If is_constraint: clamp to [0, +inf) like your old version
        const double J = (is_constraint ? std::max(0.0, J_raw) : J_raw);
        ret[0] = J;

        // -------------------------
        // 4) Gradient (only when "active")
        //    Your old grad returns 0 when J_constraint<=0
        // -------------------------
        // If you use is_constraint=true, gradient only when J_raw>0 (i.e. active)
        const bool active = (J_raw > 1.0);  // use raw to decide activity, consistent with old code
        if (!active) {
            // ret[1], ret[2] already zero
            return ret;
        }

        // softmax-weighted normal: n_bar = (w^T N)/sum(w)
        // NOTE: w uses shifted exp; weights are same as unshifted.
        const Eigen::RowVector2d n_bar = (w.matrix().transpose() * N) / denom; // 1x2

        // M = d(x_cap)/d(x)??  You used ComputeSCPx(x-center) in old code.
        const Eigen::Matrix2d M = ComputeSCPx(xc);

        // old: g = -(3*J^2) * (n_bar * M)^T
        // Use J (clamped) or J_raw? Your old code used J_constraint (already >0).
        const double J_use = (is_constraint ? J : J_raw);

        Eigen::Vector2d g = -(3.0 * J_use * J_use) * (n_bar * M).transpose();

        ret[1] = g.x();
        ret[2] = g.y();
        return ret;
    }

    Eigen::Vector2d StarConvexHandler::gradOfSCPConstraint(const double& J_constraint, const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices)
    {
        Eigen::Vector2d x_cap = center + pointTransform(x - center);

        int n = (int)ch_vertices.size();
        Eigen::MatrixXd N(n, 2);
        Eigen::ArrayXd d(n);

        for(int i=0;i<n;i++){
            // auto edge = std::make_pair(ch_vertices[i], ch_vertices[(i+1)%n]);

            N.row(i) = getOuterNormalFactor({ch_vertices[i], ch_vertices[(i+1)%n]}, center).transpose();      // 每条边各自的 normal
            d(i) = getPoint2PlaneDistance(x_cap, {ch_vertices[i], ch_vertices[(i+1)%n]}, center);
        }

        Eigen::Vector2d g = Eigen::Vector2d::Zero();
        if(J_constraint > 0){
            Eigen::ArrayXd w = (alpha * d).exp();   // softmax 权重
            double denom = w.sum();

            Eigen::RowVector2d n_bar = (w.matrix().transpose() * N) / denom;  // 1×2

            Eigen::Matrix2d M = ComputeSCPx(x - center);

            g = -(3.0 * J_constraint * J_constraint) * (n_bar * M).transpose();
        }
        return g;
    }

    Eigen::Matrix2d StarConvexHandler::ComputeSCPx(const Eigen::Vector2d& x) {
        const double n = x.norm();
        // 避免 n=0 导致除零；你也可以按需求改成 throw / assert
        const double eps = 1e-12;
        const double n_safe = std::max(n, eps);

        const double n2 = n_safe * n_safe;
        const double n3 = n2 * n_safe;

        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

        // inside = ||x||^2 I - x x^T - (||x||^3)/(2r) I
        Eigen::Matrix2d inside = n2 * I - (x * x.transpose()) - (n3 / (2.0 * flipRadius)) * I;

        // result = (r / ||x||^3) * inside
        Eigen::Matrix2d result = 2 * (flipRadius / n3) * inside;
        return result;        
    }

    double StarConvexHandler::sensitivityCost(const Eigen::Vector2d& x, const Eigen::Vector2d& center, const std::vector<Eigen::Vector2d>& ch_vertices) {
        // Operate point transformation
        Eigen::Vector2d x_cap = center + pointTransform(x - center);
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
        double theta_max = acos(cos_thetas(idx_max));
        // std::cout << "distances max = " << distances(idx_max) 
        //           << ", sensitivity max = " << theta_max
        //           << ", idx_max = " << idx_max 
        //           << std::endl;
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


    // 计算给定凸多边形（顶点集合）在方向 d 上的支撑函数 h_C(d) = max_v d^T v
    double StarConvexHandler::supportFunction(const std::vector<Vec2>& poly, const Vec2& d) {
        double max_val = -std::numeric_limits<double>::infinity();
        for (const auto& v : poly) {
            double val = d.dot(v);
            if (val > max_val) max_val = val;
        }
        return max_val;
    }

    // 输入：原凸多边形顶点（任意顺序，只要“凸”即可）
    //      num_directions：采样方向数量（越大越贴近原多边形，但顶点也会更多）
    // 输出：一个新的凸多边形顶点集合，保证包含原多边形
    std::vector<Vec2> StarConvexHandler::approximateOuterHull(const std::vector<Vec2>& original_poly,
                                                int num_directions) {
        if (original_poly.size() < 3) {
            throw std::runtime_error("original_poly must have at least 3 points.");
        }
        if (num_directions < 3) {
            throw std::runtime_error("num_directions must be >= 3.");
        }

        // 1. 生成一圈均匀分布的方向 d_k
        std::vector<Vec2> directions;
        directions.reserve(num_directions);
        const double TWO_PI = 2.0 * M_PI;

        for (int k = 0; k < num_directions; ++k) {
            double theta = TWO_PI * static_cast<double>(k) / static_cast<double>(num_directions);
            directions.emplace_back(std::cos(theta), std::sin(theta));
        }

        // 2. 对每个方向 d_k 计算支撑函数值 s_k = max_v d_k^T v
        std::vector<double> offsets;
        offsets.reserve(num_directions);
        for (int k = 0; k < num_directions; ++k) {
            double h = supportFunction(original_poly, directions[k]);
            offsets.push_back(h);
        }

        // 3. 对相邻两条直线 d_k^T x = s_k 和 d_{k+1}^T x = s_{k+1} 求交点
        //    注意最后一条和第一条也要相交（环）
        std::vector<Vec2> new_poly;
        new_poly.reserve(num_directions);

        for (int k = 0; k < num_directions; ++k) {
            int k_next = (k + 1) % num_directions;

            const Vec2& d1 = directions[k];
            const Vec2& d2 = directions[k_next];
            double s1 = offsets[k];
            double s2 = offsets[k_next];

            // 直线形式：
            // d1.x * x + d1.y * y = s1
            // d2.x * x + d2.y * y = s2
            Eigen::Matrix2d A;
            A(0, 0) = d1.x(); A(0, 1) = d1.y();
            A(1, 0) = d2.x(); A(1, 1) = d2.y();
            Eigen::Vector2d b(s1, s2);

            double det = A.determinant();
            if (std::fabs(det) < 1e-9) {
                // 两个方向几乎平行，跳过这个交点
                continue;
            }

            Vec2 x = A.colPivHouseholderQr().solve(b);
            new_poly.push_back(x);
        }

        return new_poly;
    }
}

void publishPolygonMarker(const std::vector<Eigen::Vector2d> &vertices)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "convexhull_new";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    // 线宽
    marker.scale.x = 0.5;

    // 黄色
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.0);

    // 塞点
    marker.points.clear();
    for (const auto &v : vertices)
    {
        geometry_msgs::Point p;
        p.x = v.x();
        p.y = v.y();
        p.z = 0.0;
        marker.points.push_back(p);
    }

    // 闭合多边形
    if (!marker.points.empty())
        marker.points.push_back(marker.points.front());

    ros::Publisher pubConvexhullMarker;
    pubConvexhullMarker.publish(marker);
}

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "convex_hull_subscriber");
//     ros::NodeHandle nh;
    // StarConvexHandler chh(nh);
    // ros::param::get("/flip_radius", flipRadius);
    // ros::param::get("/alpha_relax", alpha);
    // ros::param::get("/visiable_range", visiableRange);
    // ROS_INFO("SCP radius: %f, visiable range: %f", flipRadius, visiableRange);

    // ros::param::get("/d_los_max", flipD);
    // ros::param::get("/d_los_min", flipD_min);
    // ros::param::get("/k_omega", kS);
    // ros::param::get("/m_dim", m_d_min);
    // ros::param::get("/m_lambda", m_lambda);
    // ros::Rate rate(10);
    // ros::Publisher pub_pts = nh.advertise<visualization_msgs::Marker>("three_points", 1);
    // ros::Publisher pubConvexhullMarker = nh.advertise<visualization_msgs::Marker>("convexhull_new", 2);
    // while (ros::ok())
    // {
    //     auto vertices = chh.getVertices();
    //     if (vertices.empty()) {
    //         ros::spinOnce();
    //         rate.sleep();
    //         continue;
    //     }
    //     std::vector<geometry_msgs::Point> pts;

    //     geometry_msgs::Point p;
    //     p.z = 0.0;
    //     Eigen::Vector2d x = {4, -9};
    //     Eigen::Vector2d center = {0, 0};
    //     p.x = x(0); p.y = x(1); pts.push_back(p);
    //     auto x_cap = chh.pointTransform(x);
    //     p.x = x_cap(0); p.y = x_cap(1); pts.push_back(p);
    //     p.x = x(0); p.y = x(1); pts.push_back(p);       
    //     chh.starConvexConstraint(x, center, vertices);
    //     chh.sensitivityCost(x, center, vertices);
    //     chh.drawPoints(pub_pts, pts);
    //     auto approx_poly = chh.approximateOuterHull(vertices, 20);
    //     publishPolygonMarker(approx_poly);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
//     ros::spin();
//     return 0;
// }
