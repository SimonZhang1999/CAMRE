#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <boost/bind.hpp>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

std::string name_space;
ros::Publisher filtered_scan_pub;
std::vector<std::string> robot_names;

message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_1;
tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_1;

tf::TransformListener* listener;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    // Initialize the filtered scan message
    sensor_msgs::LaserScan filtered_scan = *scan_in;

    // Get the robot's own transform
    tf::StampedTransform transform;
    std::vector<std::pair<float, float>> otherPositions;
    tf::Vector3 relativeTranslation;
    for (auto& otherName: robot_names) {
        if (otherName.substr(0, 7) == name_space) {
            continue;
        }
        try {
            listener->lookupTransform(name_space+"/base_laser_link", otherName + "base_laser_link", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }
        relativeTranslation = transform.getOrigin();
        otherPositions.push_back({relativeTranslation.getX(), relativeTranslation.getY()});
        std::string targetFrame = otherName + "base_laser_link";
        // ROS_INFO("%s position: %f, %f", targetFrame.c_str(), relativeTranslation.getX(), relativeTranslation.getY());
    }


    // Loop through each scan point
    for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
        float range = scan_in->ranges[i];

        // Calculate the x and y position of the scan point in the robot's frame
        float angle = scan_in->angle_min + i * scan_in->angle_increment;
        float x = range * cos(angle);
        float y = range * sin(angle);

        // Check if the point is within the robot's body
        // Adjust these thresholds based on your robot's size
        for (const auto& otherPos: otherPositions) {
            if (std::abs(otherPos.first - x) < 0.2 && std::abs(otherPos.second - y) < 0.2) {
                filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
                break;
            }
        }
    }
    // Publish the filtered scan
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create a transform listener
    listener = new tf::TransformListener;


    name_space = nh_private.param("namespace", std::string("robot_1"));

    robot_names.assign({"robot_1/", "robot_2/", "robot_3/", "robot_4/"});


    scan_filter_sub_1 = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "base_scan", 5);
    scan_filter_1 = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_1, *listener, name_space+"/odom", 5);
    scan_filter_1->registerCallback(boost::bind(scanCallback, _1));

    // ros::Subscriber scan_sub = nh.subscribe("base_scan", 1000, scanCallback);
    filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("filtered_scan", 1000);


    ros::spin();

    // 清理
    delete scan_filter_sub_1;
    delete scan_filter_1;

    return 0;
}
