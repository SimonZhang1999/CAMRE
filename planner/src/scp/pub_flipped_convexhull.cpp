/**
 * @file pub_flipped_convexhull.cpp
 * @author weijian (wxz163@student.bham.ac.uk)
 * @brief Subscribe laserScan, derive visibility constraints, take derivative, and then move following robot.
 * This file also visualize robot_2 before and after flipping w.r.t. robot_1, for debugging.
 * @version 0.1
 * @date 2025-12-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h> // 包含Float32消息类型
#include <gazebo_msgs/ModelStates.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Eigen>

std::string name_space;

float visiableRange; // Used to add augmented points into raw pointCloud
float flipRadius;    // Radius for spherical flipping
float testPointX, testPointY;
float alpha; // Coefficient in log-exp-sum relaxation

float flipD;     // Safe distance (to convexHull) to ignore visible constraints
float flipD_min; // Mimimum allowed distance to convexHull to maintain visible
float kS;        // Constant of the weight function

std::vector<float> robot_color; // The color to show visible region

geometry_msgs::PointStamped testPointInOdom;
tf2_ros::Buffer tfBuffer;

std::mutex testPointMutex;
cv::Point2f testPoint;

ros::Publisher pubScpBoundary;
ros::Publisher pubScpVertices;
ros::Publisher pubReflectPoints;
ros::Publisher pubAugmentPoints;
ros::Publisher pubMaxDistance;
ros::Publisher pubTestMarker;
ros::Publisher pubConvexhullVertices;
ros::Publisher pubConvexhullMarker;
ros::Publisher pubConvexVertices;
ros::Publisher pub_cloud;
// std::vector<Eigen::Vector2f> points_front;
// std::vector<Eigen::Vector2f> points_back_in_front;
bool front_ready = false;
bool back_ready  = false;
std::mutex scan_mutex;

/**
 * @brief Perform spherical flipping operation.
 *
 * @param flippedPoint
 * @return cv::Point2f
 */
cv::Point2f sphericalFlipping(const cv::Point2f &flippedPoint, const double flipRadius)
{
    float norm = cv::norm(flippedPoint);
    // 映射函数 p' = 2R - p
    float newX = 2 * flipRadius * flippedPoint.x / norm - flippedPoint.x;
    float newY = 2 * flipRadius * flippedPoint.y / norm - flippedPoint.y;
    return cv::Point2f{newX, newY};
}

/**
 * @brief Use log-exp-sum relaxation to find the maximum distance in the vector.
 *
 * @param distances
 * @return double
 */
double findMaximum(const std::vector<float> &distances)
{
    double sumAll = 0;
    for (float dist : distances)
    {
        sumAll += std::exp(alpha * dist);
    }
    return std::log(sumAll) / alpha;
}

// Interpolate two points according to the given step
std::vector<cv::Point2f> interpolatePoints(const cv::Point2f &pt1, const cv::Point2f &pt2, float step)
{
    std::vector<cv::Point2f> points;
    points.push_back(pt1);

    float distance = cv::norm(pt2 - pt1);
    if (distance <= step)
    {
        points.push_back(pt2);
        return points;
    }
    cv::Point2f direction = (pt2 - pt1) / distance;
    float numSteps = std::floor(distance / step);

    for (int i = 1; i <= numSteps; ++i)
    {
        cv::Point2f newPoint = pt1 + direction * step * i;
        points.push_back(newPoint);
    }
    points.push_back(pt2);
    return points;
}

void visualizeAugmentedPointCloud(const std::vector<Eigen::Vector2f> &data)
{
    visualization_msgs::Marker pointMarker1;
    pointMarker1.header.frame_id = name_space + "/base_link";

    pointMarker1.header.stamp = ros::Time::now();
    pointMarker1.ns = "points";
    pointMarker1.id = 3;
    pointMarker1.type = visualization_msgs::Marker::POINTS;
    pointMarker1.action = visualization_msgs::Marker::ADD;
    pointMarker1.pose.orientation.w = 1.0;
    pointMarker1.scale.x = 0.2; // 点的大小
    pointMarker1.scale.y = 0.2; // 点的大小
    pointMarker1.color.r = 1.0;
    pointMarker1.color.g = 1.0;
    pointMarker1.color.b = 0.2;
    pointMarker1.color.a = 1.0;
    for (const auto &point : data)
    {
        geometry_msgs::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.0;
        pointMarker1.points.push_back(p);
    }
    pubAugmentPoints.publish(pointMarker1);
}

void visualizeVisibleRegionVertices(const std::vector<cv::Point2f> &vertexData)
{
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = name_space + "/base_link";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "points";
    pointMarker.id = 0;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.scale.x = 0.2; // 点的大小
    pointMarker.scale.y = 0.2; // 点的大小
    pointMarker.color.r = 0.5;
    pointMarker.color.g = 0.5;
    pointMarker.color.b = 0.0;
    pointMarker.color.a = 0.7;
    for (const auto &point : vertexData)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        pointMarker.points.push_back(p);
    }
    pubScpVertices.publish(pointMarker);
}

void visualizeVisibleRegionBoundary(const std::vector<cv::Point2f> &flipVertexData)
{
    visualization_msgs::Marker lineMarker;
    lineMarker.header.frame_id = name_space + "/base_link";
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.ns = "lines";
    lineMarker.id = 1;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.action = visualization_msgs::Marker::ADD;
    lineMarker.pose.orientation.w = 1.0;
    lineMarker.scale.x = 0.15; // default 0.1, linewidth  0.05; 0.1用于exploration更清晰； 0.02 for iot demo
    // lineMarker.color.r = robot_color[0];
    // lineMarker.color.g = robot_color[1];
    // lineMarker.color.b = robot_color[2];
    // lineMarker.color.a = 1; // 0.6
    if (name_space == "robot_1") {
        lineMarker.color.r = 1.0;
        lineMarker.color.g = 0.0;
        lineMarker.color.b = 0.0;
        lineMarker.color.a = 0.6; // 0.6
    }
    if (name_space == "robot_2") {
        lineMarker.color.r = 0.0;
        lineMarker.color.g = 1.0;
        lineMarker.color.b = 0.0;
        lineMarker.color.a = 0.6; // 0.6
    }
    if (name_space == "robot_3") {
        lineMarker.color.r = 0.0;
        lineMarker.color.g = 0.0;
        lineMarker.color.b = 1.0;
        lineMarker.color.a = 0.6; // 0.6
    }

    if (name_space == "robot_4") {
        lineMarker.color.r = 1.0;
        lineMarker.color.g = 0.0;
        lineMarker.color.b = 1.0;
        lineMarker.color.a = 0.6; // 0.6
    }

    if (name_space == "robot_4") {
        lineMarker.color.r = 0.0;
        lineMarker.color.g = 1.0;
        lineMarker.color.b = 1.0;
        lineMarker.color.a = 0.6; // 0.6
    }

    float insert_step = 0.5;
    for (int i = 0; i < flipVertexData.size(); i++)
    {
        int i_next = (i + 1) % flipVertexData.size();
        std::vector<cv::Point2f> interpolateEdge = interpolatePoints(flipVertexData[i], flipVertexData[i_next], insert_step);
        // Pop last element
        interpolateEdge.pop_back();
        for (const cv::Point2f &point : interpolateEdge)
        {
            cv::Point2f innerPoint = sphericalFlipping(point, flipRadius);
            geometry_msgs::Point p;
            p.x = innerPoint.x;
            p.y = innerPoint.y;
            p.z = 0.0;
            lineMarker.points.push_back(p);
        }
    }
    pubScpBoundary.publish(lineMarker);
}

void visualizeTestPoints(const cv::Point2f &testPoint, const cv::Point2f &flipTestPoint)
{
    // 可视化testPoint和flipTestPoint到base_laser_link坐标系下
    visualization_msgs::Marker testMarker;
    testMarker.header.frame_id = name_space + "/base_link";
    testMarker.header.stamp = ros::Time::now();
    testMarker.ns = "points";
    testMarker.id = 10;
    testMarker.type = visualization_msgs::Marker::POINTS;
    testMarker.action = visualization_msgs::Marker::ADD;
    testMarker.pose.orientation.w = 1.0;
    testMarker.scale.x = 0.8; // 点的大小
    testMarker.scale.y = 0.8; // 点的大小
    testMarker.color.r = 1;
    testMarker.color.g = 0.0;
    testMarker.color.b = 0.0;
    testMarker.color.a = 1.0;

    geometry_msgs::Point p1;
    p1.x = testPoint.x;
    p1.y = testPoint.y;
    p1.z = 0.0;
    testMarker.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = flipTestPoint.x;
    p2.y = flipTestPoint.y;
    p2.z = 0.0;
    testMarker.points.push_back(p2);
    pubTestMarker.publish(testMarker);
}

/**
 * @brief Take rawPoints, output flipped points and vertices of the flipped convex hull
 *
 * @param rawPoints the list of raw laser points
 * @param vertexData the list of raw laser points that corresponds to vertices of the flipped convexhull
 * @param flippedConvexVertex the list of vertices of the flipped convexhull
 */
void fromDataToSCP(std::vector<Eigen::Vector2f> &rawPoints, std::vector<cv::Point2f> &vertexData, std::vector<cv::Point2f> &flippedConvexVertex)
{
    // Flip all data in the original pointcloud,翻转后，最外部的点，对应翻转前距离原点最近的点
    std::vector<cv::Point2f> flipData(rawPoints.size(), cv::Point2f(0, 0)); // flipData里面包含了(0, 0)
    for (size_t i = 0; i < rawPoints.size(); i++)
    {
        cv::Point2f data_i{rawPoints[i](0), rawPoints[i](1)};
        flipData[i] = sphericalFlipping(data_i, flipRadius);
    }

    std::vector<int> vertexIndice; // 下标集合，指向构成convexHull顶点的点的下标
    // Return hull vertex indices in counter-clockwise order
    cv::convexHull(flipData, vertexIndice, false, false);

    // vertexData 保存convexHull顶点对应的原数据点，即starConvexRegion的顶点
    for (size_t i = 0; i < vertexIndice.size(); i++)
    {
        int v = vertexIndice[i]; // convexHull对应的下标
        vertexData.push_back(cv::Point2f(rawPoints[v](0), rawPoints[v](1)));
        flippedConvexVertex.push_back(cv::Point2f(flipData[v].x, flipData[v].y));
    }

    // Visualize flipped points
    visualization_msgs::Marker pointMarker;
    pointMarker.header.frame_id = name_space + "/base_link";
    pointMarker.header.stamp = ros::Time::now();
    pointMarker.ns = "points";
    pointMarker.id = 2;
    pointMarker.type = visualization_msgs::Marker::POINTS;
    pointMarker.action = visualization_msgs::Marker::ADD;
    pointMarker.pose.orientation.w = 1.0;
    pointMarker.scale.x = 0.5; // 点的大小
    pointMarker.scale.y = 0.5; // 点的大小
    pointMarker.color.r = 0.2;
    pointMarker.color.g = 0.5;
    pointMarker.color.b = 0.0;
    pointMarker.color.a = 0.6;
    for (const auto &point : flipData)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        pointMarker.points.push_back(p);
    }

    // Visualize flipped convexHull
    visualization_msgs::Marker lineMarker;
    lineMarker.header.frame_id = name_space + "/base_link";
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.ns = "lines";
    lineMarker.id = 33;
    lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
    lineMarker.action = visualization_msgs::Marker::ADD;
    lineMarker.pose.orientation.w = 1.0;
    lineMarker.scale.x = 0.5; // 线的宽度
    lineMarker.color.r = 0.0;
    lineMarker.color.g = 0.5;
    lineMarker.color.b = 1.0;
    lineMarker.color.a = 0.3;
    for (int i = 0; i <= vertexIndice.size(); i++)
    {
        int real_i = i % vertexIndice.size();
        geometry_msgs::Point p;
        p.x = flipData[vertexIndice[real_i]].x;
        p.y = flipData[vertexIndice[real_i]].y;
        p.z = 0.0;
        lineMarker.points.push_back(p);
    }

    pubReflectPoints.publish(pointMarker);
    pubReflectPoints.publish(lineMarker);
    return;
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    // Find the index of robot_2 in the ModelStates message
    auto it = std::find(msg->name.begin(), msg->name.end(), "robot_2");
    if (it != msg->name.end())
    {
        int index = std::distance(msg->name.begin(), it); // 计算两个迭代器之间的距离
        geometry_msgs::Pose robot2_pose = msg->pose[index];
        testPointMutex.lock();
        testPoint.x = robot2_pose.position.x;
        testPoint.y = robot2_pose.position.y;
        testPointMutex.unlock();
    }
    else
    {
        ROS_WARN("robot_2 not found in the model states.");
    }
}

void publishPolygonMarker(const std::vector<Eigen::Vector2d> &vertices)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "convexhull";
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

    pubConvexhullMarker.publish(marker);
}

void ScanHandler(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    ROS_DEBUG("Range nums: %d", int(scan->ranges.size())); // 901

    float angleMin = scan->angle_min;
    float angleMax = scan->angle_max;
    float angleIncre = scan->angle_increment;
    float rangeMax = scan->range_max;

    float currAngle = angleMin;
    std::vector<Eigen::Vector2f> data;
    for (float dist : scan->ranges)
    { // transform laserPoint into robot's local coordinate frame
        if (dist == 2 * rangeMax)
        { // Omit laser points that hit on the robot's body
            currAngle += angleIncre;
            continue;
        }
        else if (dist > 2 * rangeMax)
        { // these are the rays do not hit anything
            dist = std::max(visiableRange, rangeMax);
        }
        float px = dist * std::cos(currAngle);
        float py = dist * std::sin(currAngle);
        data.push_back(Eigen::Vector2f(px, py));
        currAngle += angleIncre;
    }

    std::vector<cv::Point2f> vertexData;
    std::vector<cv::Point2f> flipVertexData;
    fromDataToSCP(data, vertexData, flipVertexData);

    // Visualization of the flipped points
    visualizeAugmentedPointCloud(data);
    visualizeVisibleRegionVertices(vertexData);
    visualizeVisibleRegionBoundary(flipVertexData);

    // testPoint is in odom frame; SCP constraints are in master robot's local frame
    // if (name_space == "robot_1")
    // {
    //     geometry_msgs::PointStamped testPointInOdom;
    //     testPointInOdom.header.frame_id = "odom";
    //     testPointInOdom.header.stamp = ros::Time::now();
    //     testPointMutex.lock();
    //     testPointInOdom.point.x = testPoint.x;
    //     testPointInOdom.point.y = testPoint.y;
    //     testPointInOdom.point.z = 0;
    //     testPointMutex.unlock();

    //     geometry_msgs::PointStamped testPointInBaselink;
    //     try
    //     {
    //         geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(name_space + "/sensor_laser", "odom", ros::Time(0), ros::Duration(3.0));
    //         tf2::doTransform(testPointInOdom, testPointInBaselink, transformStamped);
    //         ROS_DEBUG("odom: (%.2f, %.2f, %.2f) -----> base_laser_link: (%.2f, %.2f, %.2f) at time %.2f",
    //                   testPointInOdom.point.x, testPointInOdom.point.y, testPointInOdom.point.z,
    //                   testPointInBaselink.point.x, testPointInBaselink.point.y, testPointInBaselink.point.z,
    //                   testPointInBaselink.header.stamp.toSec());
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_laser_link\": %s", ex.what());
    //     }
    //     cv::Point2f testPointLocal{testPointInBaselink.point.x, testPointInBaselink.point.y};
    //     cv::Point2f flipTestPoint = sphericalFlipping(testPointLocal, flipRadius);
    //     visualizeTestPoints(testPointLocal, flipTestPoint);
    // }

    // Broadcast visible convexhull to other ROS nodes
    // 如果你还要发布 convexhull 顶点 (Float32MultiArray)，
    geometry_msgs::TransformStamped tf_f2m;
    try
    {
        tf_f2m = tfBuffer.lookupTransform(
            "map",                // 目标坐标系
            name_space + "/base_link",   // 源坐标系（flipVertexData 当前所在的 frame）
            ros::Time(0),
            ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(1.0,
                        "TF lookupTransform base_link->map failed: %s",
                        ex.what());
        return;
    }

    std::vector<float> flipVertexDataArray;
    std::vector<Eigen::Vector2d> convex_vertices;
    flipVertexDataArray.reserve(flipVertexData.size() * 2);

    for (const cv::Point2f &p_f : flipVertexData)
    {
        geometry_msgs::PointStamped pt_f, pt_m;
        pt_f.header.frame_id = name_space + "/base_link";
        pt_f.header.stamp    = ros::Time(0);   // 或者用最近的 scan 时间
        pt_f.point.x = p_f.x;
        pt_f.point.y = p_f.y;
        pt_f.point.z = 0.0;

        tf2::doTransform(pt_f, pt_m, tf_f2m);

        // 存 map 坐标系下的凸包顶点
        flipVertexDataArray.push_back(static_cast<float>(pt_m.point.x));
        flipVertexDataArray.push_back(static_cast<float>(pt_m.point.y));
        convex_vertices.push_back({pt_m.point.x, pt_m.point.y});
    }
    // 注意：原来你把 scan->intensities[0/1] 塞进去，这里已经没有 scan 指针了
    // 如果那俩值真的有用，你可以在上面合并 all_points 之前另外存下来再 push_back。
    // Add closest laser point to the robot convexhull data structure, for reuse of the callback function
    // flipVertexDataArray.push_back(scan->intensities[0]);
    // flipVertexDataArray.push_back(scan->intensities[1]);
    // 创建 Float32MultiArray 消息
    publishPolygonMarker(convex_vertices);
    std_msgs::Float32MultiArray msg;
    msg.data = flipVertexDataArray;
    pubConvexhullVertices.publish(msg);
    // std::vector<float> flipVertexDataArray;
    // for (const cv::Point2f &point : flipVertexData)
    // {
    //     flipVertexDataArray.push_back(point.x);
    //     flipVertexDataArray.push_back(point.y);
    // }
    // Add closest laser point to the robot convexhull data structure, for reuse of the callback function
    // flipVertexDataArray.push_back(scan->intensities[0]);
    // flipVertexDataArray.push_back(scan->intensities[1]);
    // 创建 Float32MultiArray 消息
    // std_msgs::Float32MultiArray msg;
    // msg.data = flipVertexDataArray;
    // pubConvexhullVertices.publish(msg);
    sensor_msgs::PointCloud2 cloud;

    // 1) 不做坐标变换：点云在 scan_msg->header.frame_id 下
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, cloud);

    // 2) 如果你想把点云投到某个固定坐标系（例如 odom/base_link/map）
    //    用 transformLaserScanToPointCloud（需要 TF）
    // try {
    //   projector_.transformLaserScanToPointCloud("odom", *scan_msg, cloud, tfBuffer_);
    // } catch (tf2::TransformException& ex) {
    //   ROS_WARN("TF failed: %s", ex.what());
    //   return;
    // }

    pub_cloud.publish(cloud);
    return;
}


// void ScanHandler(const sensor_msgs::LaserScan::ConstPtr &scan)
// {
//     float angleMin   = scan->angle_min;
//     float angleIncre = scan->angle_increment;

//     float currAngle = angleMin;

//     std::vector<Eigen::Vector2f> local_points;
//     local_points.reserve(scan->ranges.size());

//     for (float dist : scan->ranges)
//     {
//         // 建议用更稳的过滤：
//         if (!std::isfinite(dist) || dist < scan->range_min || dist > scan->range_max)
//         {
//             currAngle += angleIncre;
//             continue;
//         }

//         float px = dist * std::cos(currAngle);
//         float py = dist * std::sin(currAngle);
//         local_points.emplace_back(px, py);

//         currAngle += angleIncre;
//     }

//     {
//         std::lock_guard<std::mutex> lock(scan_mutex);

//         // 1) 前激光：直接存成 robot_1/front_laser_link 下的点
//         if (scan->header.frame_id == "robot_1/front_laser_link")
//         {
//             points_front = std::move(local_points);
//             front_ready = true;
//         }
//         // 2) 后激光：先转成 robot_1/front_laser_link 下的点，再存
//         else if (scan->header.frame_id == "robot_1/back_laser_link")
//         {
//             points_back_in_front.clear();
//             points_back_in_front.reserve(local_points.size());

//             // 查询一次 TF: robot_1/front_laser_link <- back_laser_link
//             geometry_msgs::TransformStamped tf_b2f;
//             try
//             {
//                 tf_b2f = tfBuffer.lookupTransform("robot_1/front_laser_link",
//                                                   "robot_1/back_laser_link",
//                                                   ros::Time(0),  // 用最近的变换
//                                                   ros::Duration(0.1));
//             }
//             catch (tf2::TransformException &ex)
//             {
//                 ROS_WARN_THROTTLE(1.0,
//                                   "TF lookupTransform back_laser_link->robot_1/front_laser_link failed: %s",
//                                   ex.what());
//                 return;
//             }

//             // 把每个点从 back_laser_link 转到 robot_1/front_laser_link
//             for (const auto &p_b : local_points)
//             {
//                 geometry_msgs::PointStamped pt_b, pt_f;
//                 pt_b.header.frame_id = "robot_1/back_laser_link";
//                 pt_b.header.stamp    = scan->header.stamp;
//                 pt_b.point.x = p_b.x();
//                 pt_b.point.y = p_b.y();
//                 pt_b.point.z = 0.0;

//                 tf2::doTransform(pt_b, pt_f, tf_b2f);

//                 points_back_in_front.emplace_back(pt_f.point.x, pt_f.point.y);
//             }

//             back_ready = true;
//         }
//         else
//         {
//             ROS_WARN_THROTTLE(1.0, "Scan in unexpected frame: %s",
//                               scan->header.frame_id.c_str());
//             return;
//         }
//     }

//     // 等两边都 ready 再做 SCP
//     {
//         std::lock_guard<std::mutex> lock(scan_mutex);
//         if (!front_ready || !back_ready)
//             return;

//         std::vector<Eigen::Vector2f> all_points;
//         all_points.reserve(points_front.size() + points_back_in_front.size());
//         all_points.insert(all_points.end(), points_front.begin(), points_front.end());
//         all_points.insert(all_points.end(), points_back_in_front.begin(), points_back_in_front.end());

//         front_ready = false;
//         back_ready  = false;

//         // ===== 下面就是你原来 ScanHandler 里的逻辑 =====
//         std::vector<cv::Point2f> vertexData;
//         std::vector<cv::Point2f> flipVertexData;
//         fromDataToSCP(all_points, vertexData, flipVertexData);

//         visualizeAugmentedPointCloud(all_points);
//         visualizeVisibleRegionVertices(vertexData);
//         visualizeVisibleRegionBoundary(flipVertexData);

//         // 如果你还要发布 convexhull 顶点 (Float32MultiArray)，
//         // 这里照旧用 flipVertexData 来打包即可
//         // 先查一次 robot_1/front_laser_link -> map 的变换
//         geometry_msgs::TransformStamped tf_f2m;
//         try
//         {
//             tf_f2m = tfBuffer.lookupTransform(
//                 "map",                // 目标坐标系
//                 "robot_1/front_laser_link",   // 源坐标系（flipVertexData 当前所在的 frame）
//                 ros::Time(0),
//                 ros::Duration(0.1));
//         }
//         catch (tf2::TransformException &ex)
//         {
//             ROS_WARN_THROTTLE(1.0,
//                             "TF lookupTransform robot_1/front_laser_link->map failed: %s",
//                             ex.what());
//             return;
//         }

//         std::vector<float> flipVertexDataArray;
//         std::vector<Eigen::Vector2d> convex_vertices;
//         flipVertexDataArray.reserve(flipVertexData.size() * 2);

//         for (const cv::Point2f &p_f : flipVertexData)
//         {
//             geometry_msgs::PointStamped pt_f, pt_m;
//             pt_f.header.frame_id = "robot_1/front_laser_link";
//             pt_f.header.stamp    = ros::Time(0);   // 或者用最近的 scan 时间
//             pt_f.point.x = p_f.x;
//             pt_f.point.y = p_f.y;
//             pt_f.point.z = 0.0;

//             tf2::doTransform(pt_f, pt_m, tf_f2m);

//             // 存 map 坐标系下的凸包顶点
//             flipVertexDataArray.push_back(static_cast<float>(pt_m.point.x));
//             flipVertexDataArray.push_back(static_cast<float>(pt_m.point.y));
//             convex_vertices.push_back({pt_m.point.x, pt_m.point.y});
//         }
//         // 注意：原来你把 scan->intensities[0/1] 塞进去，这里已经没有 scan 指针了
//         // 如果那俩值真的有用，你可以在上面合并 all_points 之前另外存下来再 push_back。
//         // Add closest laser point to the robot convexhull data structure, for reuse of the callback function
//         // flipVertexDataArray.push_back(scan->intensities[0]);
//         // flipVertexDataArray.push_back(scan->intensities[1]);
//         // 创建 Float32MultiArray 消息
//         publishPolygonMarker(convex_vertices);
//         std_msgs::Float32MultiArray msg;
//         msg.data = flipVertexDataArray;
//         pubConvexhullVertices.publish(msg);
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_flipped_convexhull");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::param::get("/flip_radius", flipRadius);
    ros::param::get("/visiable_range", visiableRange);
    ROS_INFO("SCP radius: %f, visiable range: %f", flipRadius, visiableRange);

    ros::param::get("/d_los_max", flipD);
    ros::param::get("/d_los_min", flipD_min);
    ros::param::get("/k_omega", kS);

    float color_r = nh_private.param("color_r", 1.0);
    float color_g = nh_private.param("color_g", 1.0);
    float color_b = nh_private.param("color_b", 1.0);
    robot_color.assign({color_r, color_g, color_b});
    ROS_INFO("Set robot color: %f(r), %f(g), %f(b)", robot_color[0], robot_color[1], robot_color[2]);

    name_space = nh_private.param("namespace", std::string(""));
    ROS_INFO("Add name_space: %s", name_space.c_str());

    ros::Subscriber subScan = nh.subscribe<sensor_msgs::LaserScan>("filter_scan", 10, ScanHandler);
    // ros::Subscriber subModelState = nh.subscribe("/gazebo/model_states", 100, modelStatesCallback);

    pubScpBoundary = nh.advertise<visualization_msgs::Marker>("scp_boundary", 2);
    pubScpVertices = nh.advertise<visualization_msgs::Marker>("scp_vertices", 2);
    pubReflectPoints = nh.advertise<visualization_msgs::Marker>("scp_reflectPoints", 2);
    pubAugmentPoints = nh.advertise<visualization_msgs::Marker>("scp_augment", 2);
    pubTestMarker = nh.advertise<visualization_msgs::Marker>("test_points", 2);
    pubConvexhullMarker = nh.advertise<visualization_msgs::Marker>("convexhull", 2);
    pubMaxDistance = nh.advertise<std_msgs::Float32>("max_distance", 2);

    // publish ordered vertices of flipped convexHull
    pubConvexhullVertices = nh.advertise<std_msgs::Float32MultiArray>("vertices_convexhull", 2);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("filter_cloud", 10);
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Duration(5.0).sleep(); // 确保变换缓冲区已经填充

    ros::spin();
}
