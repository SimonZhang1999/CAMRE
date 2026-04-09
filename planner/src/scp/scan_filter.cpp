/**
 * @file scan_filter.cpp
 * @author weijian (wxz163@student.bham.ac.uk)
 * @brief This node has two functions:
 *      1) If a laser point hits on any robot (including itself), reset its range value as range_max
 *      2) STRICT MODE: publish filter_scan ONLY when TF from ALL other robots are available,
 *         to avoid missing robots being treated as obstacles.
 * @version 0.2
 * @date 2025-12-08
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/bind.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// -------------------------
// Globals (keep simple)
// -------------------------
static std::string name_space;                 // e.g. "robot_1"
static int robot_number = 2;
static std::vector<std::string> robot_names;  // e.g. "robot_1/", "robot_2/", ...

static ros::Publisher filtered_scan_pub;

static tf::TransformListener* listener_ptr = nullptr;
static message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sub_ptr = nullptr;
static tf::MessageFilter<sensor_msgs::LaserScan>* scan_tf_filter_ptr = nullptr;

// Params
static double threshold_self_hit  = 0.2;   // meters (radius)
static double threshold_other_hit = 0.6;   // meters (radius)
static bool strict_all_tf_required = true; // must have all other robots' TF to publish
static double warn_throttle_sec = 1.0;

// Helper: circle hit test
static inline bool hitCircle(float x, float y, float cx, float cy, float r) {
    const float dx = x - cx;
    const float dy = y - cy;
    return (dx*dx + dy*dy) <= r*r;
}

// -------------------------
// Callback
// -------------------------
static void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::LaserScan filtered_scan = *scan_in;

    // 1) Collect ALL other robots positions in this robot base_link frame
    tf::StampedTransform tf_rel;
    std::vector<std::pair<float,float>> other_positions;
    other_positions.reserve((robot_number > 0) ? (robot_number - 1) : 0);

    const std::string self_prefix = name_space + "/";

    int need = 0;
    int got  = 0;
    bool all_ok = true;

    for (const auto& other : robot_names) {
        if (other == self_prefix) continue; // skip itself exactly
        need++;

        try {
            // Transform: self base_link <- other base_link
            listener_ptr->lookupTransform(
                name_space + "/base_link",
                other + "base_link",
                ros::Time(0),
                tf_rel
            );
            const tf::Vector3 t = tf_rel.getOrigin();
            other_positions.push_back({(float)t.getX(), (float)t.getY()});
            got++;
        } catch (tf::TransformException& ex) {
            all_ok = false;
            ROS_WARN_THROTTLE(warn_throttle_sec,
                "[scan_filter] Missing TF %s/base_link <- %sbase_link : %s",
                name_space.c_str(), other.c_str(), ex.what());
        }
    }

    // 2) STRICT: publish only when we have ALL other robots transforms
    if (strict_all_tf_required && !all_ok) {
        ROS_WARN_THROTTLE(warn_throttle_sec,
            "[scan_filter] Skip publishing filter_scan (got %d/%d other robots)",
            got, need);
        return;
    }

    // 3) Filter scan points: remove self + other robots
    const float self_r  = (float)threshold_self_hit;
    const float other_r = (float)threshold_other_hit;

    for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
        const float r = scan_in->ranges[i];
        if (!std::isfinite(r)) continue;

        const float ang = scan_in->angle_min + (float)i * scan_in->angle_increment;
        const float x = r * std::cos(ang);
        const float y = r * std::sin(ang);

        // Self hit: within radius around origin (0,0) in base_link
        if (hitCircle(x, y, 0.0f, 0.0f, self_r)) {
            filtered_scan.ranges[i] = 2*scan_in->range_max;
            continue;
        }

        // Other robots hit
        bool hit_other = false;
        for (const auto& p : other_positions) {
            if (hitCircle(x, y, p.first, p.second, other_r)) {
                hit_other = true;
                break;
            }
        }
        if (hit_other) {
            filtered_scan.ranges[i] = 2*scan_in->range_max;
        }
    }

    filtered_scan_pub.publish(filtered_scan);
}

// -------------------------
// Main
// -------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_filter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Params
    pnh.param<std::string>("namespace", name_space, std::string("robot_1"));
    pnh.param("robot_number", robot_number, 2);
    pnh.param("threshold_self_hit", threshold_self_hit, 0.2);
    pnh.param("threshold_other_hit", threshold_other_hit, 0.6);
    pnh.param("strict_all_tf_required", strict_all_tf_required, true);
    pnh.param("warn_throttle_sec", warn_throttle_sec, 1.0);

    ROS_INFO("[scan_filter] namespace=%s robot_number=%d strict_all_tf_required=%s",
             name_space.c_str(), robot_number, strict_all_tf_required ? "true" : "false");

    // Build robot name list: "robot_i/"
    robot_names.clear();
    robot_names.reserve(std::max(0, robot_number));
    for (int i = 0; i < robot_number; ++i) {
        robot_names.push_back("robot_" + std::to_string(i + 1) + "/");
    }

    // TF listener
    listener_ptr = new tf::TransformListener;

    // Subscriber with TF message filter (ensures TF from scan frame -> target frame exists)
    scan_sub_ptr = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "scan", 5);
    scan_tf_filter_ptr = new tf::MessageFilter<sensor_msgs::LaserScan>(
        *scan_sub_ptr, *listener_ptr, name_space + "/base_link", 5
    );
    scan_tf_filter_ptr->registerCallback(boost::bind(scanCallback, _1));

    // Publisher
    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filter_scan", 10);

    ros::spin();

    // Cleanup
    delete scan_tf_filter_ptr; scan_tf_filter_ptr = nullptr;
    delete scan_sub_ptr;       scan_sub_ptr = nullptr;
    delete listener_ptr;       listener_ptr = nullptr;

    return 0;
}