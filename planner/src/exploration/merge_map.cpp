// /**
//  * @file merge_map.cpp
//  * @author qingchen Bi
//  * @brief replace map merge package
//  * @version 0.1
//  * @date 2025-03-31
//  * 
//  * @copyright Copyright (c) 2025
//  * 
//  */
// #include "ros/ros.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "cure_planner/PointArray.h"
// #include <tf/transform_listener.h>
// #include <boost/bind.hpp>

// using namespace std;

// #define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
// #define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

// string map_topic;
// int n_robot, robot_id;
// double map_x, map_y;
// int map_width, map_height;
// std::vector<ros::Subscriber> map_subs;
// nav_msgs::OccupancyGrid merged_map;
// nav_msgs::OccupancyGrid tmp_map;
// bool merge_init = false;
// bool rece_map = false;

// void findTF(const string parent_frame_id, const string children_frame_id, geometry_msgs::TransformStamped& transform_pose, const tf::TransformListener& listener)
// {
//     tf::StampedTransform transform_stamped;
//     // geometry_msgs::TransformStamped transform_pose;
//     // tf::Quaternion quat; 
//     // double roll,pitch,yaw; 
//     try
//     {
//             listener.waitForTransform(parent_frame_id, children_frame_id, ros::Time(0), ros::Duration(2.0));
//             listener.lookupTransform(parent_frame_id, children_frame_id, ros::Time(0), transform_stamped);
//             tf::transformStampedTFToMsg(transform_stamped, transform_pose);
//             // point_temp.x = transform_stamped.getOrigin().x();
//             // point_temp.y = transform_stamped.getOrigin().y();
//             // tf::quaternionMsgToTF(transform_pose.transform.rotation,quat); 
//             // tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
//     }
//     catch (tf::TransformException &ex)
//     {
//         ROS_ERROR("%s", ex.what());
//         ros::Duration(1.0).sleep();
//     }
// }
// void addAndDeleRobotObs(const tf::TransformListener& listener, nav_msgs::OccupancyGrid& planning_map)
// {
//     for(int k = 1; k < n_robot + 1; k++)
//     {
//         geometry_msgs::TransformStamped transform_rp;
//         findTF("map", "robot_" + to_string(k) + "/base_link", transform_rp, listener);
//         int r_global_index_x = CONTXY2DISC(transform_rp.transform.translation.x - merged_map.info.origin.position.x, merged_map.info.resolution); 
//         int r_global_index_y = CONTXY2DISC(transform_rp.transform.translation.y - merged_map.info.origin.position.y, merged_map.info.resolution);
//         if(k == robot_id)
//         {
//             for(int i = r_global_index_x - 2; i <= r_global_index_x + 2; i++)
//             {
//                 for(int j = r_global_index_y - 2; j <= r_global_index_y + 2; j++)
//                 {
//                     if(i < 0 || i >= merged_map.info.width || j < 0 || j >= merged_map.info.height)
//                         continue;
//                     else
//                     {
//                         planning_map.data[i + j * merged_map.info.width] = 0;
//                     }
//                 }
//             }
//         }
//         else
//         {
            
//             for(int i = r_global_index_x - 2; i <= r_global_index_x + 2; i++)
//             {
//                 for(int j = r_global_index_y - 2; j <= r_global_index_y + 2; j++)
//                 {
//                     if(i < 0 || i >= merged_map.info.width || j < 0 || j >= merged_map.info.height)
//                         continue;
//                     else
//                     {
//                         planning_map.data[i + j * merged_map.info.width] = 100;
//                     }
//                 }
//             }
//         }
//     }
// }
// void mergeMap(const int r_index_x, const int r_index_y, const geometry_msgs::TransformStamped transform_map)
// {
                    
//     double merge_range = 10.0;
//     int step = merge_range / tmp_map.info.resolution;
//     for(int i = r_index_x - step; i <= r_index_x + step; i++)
//     {
//         for(int j = r_index_y - step; j <= r_index_y + step; j++)
//         {
//             if(i < 0 || i >= tmp_map.info.width || j < 0 || j >= tmp_map.info.height)
//             {
//                 continue;
//             }
//             else
//             {              
//                 if(tmp_map.data[i + j * tmp_map.info.width] != -1)
//                 {
//                     geometry_msgs::Point grid_global_position;
//                     grid_global_position.x = DISCXY2CONT(i, tmp_map.info.resolution) + tmp_map.info.origin.position.x + transform_map.transform.translation.x;
//                     grid_global_position.y = DISCXY2CONT(j, tmp_map.info.resolution) + tmp_map.info.origin.position.y + transform_map.transform.translation.y;
//                     int g_global_index_x = CONTXY2DISC(grid_global_position.x - merged_map.info.origin.position.x, merged_map.info.resolution); 
//                     int g_global_index_y = CONTXY2DISC(grid_global_position.y - merged_map.info.origin.position.y, merged_map.info.resolution);
//                     if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == -1)
//                         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = tmp_map.data[i + j * tmp_map.info.width];
//                     else
//                     {
//                         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = int(0.2 * double(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width])) 
//                                                                                                         + int(0.8 * double(tmp_map.data[i + j * tmp_map.info.width]));
//                         if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] >= 80)
//                             merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
//                         else
//                             merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;
//                     }
//                     // else if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == 0 && tmp_map.data[i + j * tmp_map.info.width] == 100)
//                     // {
//                     //     bool is_merge = false;
//                     //     for(int m = i - 2; m <= i + 2;)
//                     //     {
//                     //         for(int n = j - 2; n <= j + 2;)
//                     //         {
//                     //             if(m < 0 || m >= tmp_map.info.width || n < 0 || n >= tmp_map.info.height)
//                     //                 continue;
//                     //             else
//                     //             {
//                     //                 if(tmp_map.data[m + n * tmp_map.info.width] == 100)
//                     //                 {
//                     //                     is_merge = true;
//                     //                     break;
//                     //                 }
//                     //             }
//                     //             n+=2;
//                     //         }
//                     //         m+=2;
//                     //     }
//                     //     if(is_merge)
//                     //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
//                     //     else
//                     //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;                            
//                     // }
//                     // else if(merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] == 100 && tmp_map.data[i + j * tmp_map.info.width] == 0)
//                     // {
//                     //     bool is_merge = false;
//                     //     for(int m = g_global_index_x - 2; m <= g_global_index_x + 2;)
//                     //     {
//                     //         for(int n = g_global_index_y - 2; n <= g_global_index_y + 2;)
//                     //         {
//                     //             if(m < 0 || m >= merged_map.info.width || n < 0 || n >= merged_map.info.height)
//                     //                 continue;
//                     //             else
//                     //             {
//                     //                 if(merged_map.data[m + n * merged_map.info.width] == 100)
//                     //                 {
//                     //                     is_merge = true;
//                     //                     break;
//                     //                 }
//                     //             }
//                     //             n+=2;
//                     //         }
//                     //         m+=2;
//                     //     }
//                     //     if(is_merge)
//                     //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 100;
//                     //     else
//                     //         merged_map.data[g_global_index_x + g_global_index_y * merged_map.info.width] = 0;                            
//                     // }
//                 }
//             }
//         }
//     }
// }

// void initMergedMap()
// {
//     merged_map.header.frame_id = "map";
//     merged_map.info.origin.position.x = map_x;
//     merged_map.info.origin.position.y = map_y;
//     merged_map.info.origin.position.z = -0.0;
//     merged_map.info.resolution = 0.1;
//     merged_map.info.width = map_width; 
//     merged_map.info.height = map_height; 
//     std::vector<int8_t> tmp(merged_map.info.width * merged_map.info.height, -1);
//     merged_map.data = tmp;
//     merge_init = true;    
// }
// void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg,  const std::string& topic_name)
// {
//     tmp_map = *msg;
//     rece_map = true;
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "merge_map_node");
//     ros::NodeHandle nh;
//     std::string ns;
//     ns=ros::this_node::getName();
//     ros::param::param<int>(ns + "/n_robot", n_robot, 1);
//     ros::param::param<int>(ns + "/robot_id", robot_id, 1);
//     ros::param::param<double>(ns + "/map_x", map_x, 1);
//     ros::param::param<double>(ns + "/map_y", map_y, 1);
//     ros::param::param<int>(ns + "/map_width", map_width, 1);
//     ros::param::param<int>(ns + "/map_height", map_height, 1);

//     tf::TransformListener listener;

//     for(int i = 1; i < n_robot + 1; i++)
//     {
//         map_topic = "/robot_" + std::to_string(i) + "/map";
//         ros::Subscriber map_sub =
//         nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, boost::bind(mapCallBack, _1, map_topic));
//         map_subs.push_back(map_sub);
//     }
//     ros::Publisher merged_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);
//     ros::Publisher merged_planning_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("planning_map", 10);
//     ros::Rate rate(5);

//     initMergedMap();

//     while(ros::ok())
//     {
//         if(merge_init && rece_map)
//         {
//             std::string robot_ns = tmp_map.header.frame_id;
//             int cur_id = std::stoi(robot_ns.substr(6));
//             geometry_msgs::TransformStamped transform_map, transform_robot;
//             string map_frame_id = "map";
//             string r_map_frame_id = "robot_" + to_string(cur_id) + "/map";
//             findTF(map_frame_id, r_map_frame_id, transform_map, listener);
//             string rbaselink_frame_id = "robot_" + to_string(cur_id) + "/base_link";
//             findTF(r_map_frame_id, rbaselink_frame_id, transform_robot, listener);
//             int r_index_x = CONTXY2DISC(transform_robot.transform.translation.x - tmp_map.info.origin.position.x, tmp_map.info.resolution); 
//             int r_index_y = CONTXY2DISC(transform_robot.transform.translation.y - tmp_map.info.origin.position.y, tmp_map.info.resolution);
//             mergeMap(r_index_x, r_index_y, transform_map);
//             merged_map.header.stamp = tmp_map.header.stamp;
//             // ROS_ERROR("%d, %f, %f", cur_id, transform_map.transform.translation.x, transform_map.transform.translation.y);
//             nav_msgs::OccupancyGrid planning_map;
//             planning_map = merged_map;
//             addAndDeleRobotObs(listener, planning_map);
//             if(robot_id == 1)
//                 merged_map_pub.publish(merged_map); 
//             merged_planning_map_pub.publish(planning_map);
//         }
  
//         ros::spinOnce();
//         rate.sleep();  
//     }
//     return 0;
// }

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

using namespace std;

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

static int n_robot = 3, robot_id = 1;
static double map_x = -20.0, map_y = -20.0;
static int map_width = 400, map_height = 400;

static nav_msgs::OccupancyGrid merged_map;
static bool merge_init = false;

static vector<nav_msgs::OccupancyGrid> robot_maps;   // [0..n_robot-1]
static vector<bool> robot_received;                  // 是否收到过该机器人 map

// ---------------- TF helpers ----------------
static bool findTF(const string& parent_frame_id,
                   const string& children_frame_id,
                   geometry_msgs::TransformStamped& transform_pose,
                   const tf::TransformListener& listener)
{
  tf::StampedTransform transform_stamped;
  try
  {
    listener.waitForTransform(parent_frame_id, children_frame_id, ros::Time(0), ros::Duration(0.2));
    listener.lookupTransform(parent_frame_id, children_frame_id, ros::Time(0), transform_stamped);
    tf::transformStampedTFToMsg(transform_stamped, transform_pose);
    return true;
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s -> %s, %s",
                      parent_frame_id.c_str(), children_frame_id.c_str(), ex.what());
    return false;
  }
}

static inline bool insideMerged(int gx, int gy)
{
  return (gx >= 0 && gx < (int)merged_map.info.width && gy >= 0 && gy < (int)merged_map.info.height);
}

static void initMergedMap()
{
  merged_map.header.frame_id = "map";
  merged_map.info.origin.position.x = map_x;
  merged_map.info.origin.position.y = map_y;
  merged_map.info.origin.position.z = 0.0;
  merged_map.info.resolution = 0.1;  // 你原来写死 0.1，我保持不动
  merged_map.info.width = map_width;
  merged_map.info.height = map_height;
  merged_map.data.assign(merged_map.info.width * merged_map.info.height, -1);
  merge_init = true;
}

static inline double snapDown(double x, double res) {
  return std::floor(x / res) * res;
}
static inline double snapUp(double x, double res) {
  return std::ceil(x / res) * res;
}

// 确保 merged_map 能覆盖 [minx,maxx]x[miny,maxy]（world坐标），不足就扩容并拷贝旧数据
static void ensureMergedCovers(double minx, double miny,
                               double maxx, double maxy,
                               double margin)
{
  if (!merge_init) return;

  const double res = merged_map.info.resolution;

  // 加 margin
  minx -= margin; miny -= margin;
  maxx += margin; maxy += margin;

  // 当前 merged_map world 边界
  const double cur_minx = merged_map.info.origin.position.x;
  const double cur_miny = merged_map.info.origin.position.y;
  const double cur_maxx = cur_minx + merged_map.info.width  * res;
  const double cur_maxy = cur_miny + merged_map.info.height * res;

  // 已经包含就不动
  if (minx >= cur_minx && miny >= cur_miny && maxx <= cur_maxx && maxy <= cur_maxy) {
    return;
  }

  // 新边界（对齐到栅格）
  const double new_minx = snapDown(std::min(minx, cur_minx), res);
  const double new_miny = snapDown(std::min(miny, cur_miny), res);
  const double new_maxx = snapUp  (std::max(maxx, cur_maxx), res);
  const double new_maxy = snapUp  (std::max(maxy, cur_maxy), res);

  const int new_w = (int)std::lround((new_maxx - new_minx) / res);
  const int new_h = (int)std::lround((new_maxy - new_miny) / res);

  // 防呆：避免意外爆内存（你可以按机器内存改上限）
  const long long max_cells = 120000000LL; // 1.2e8 cells ~ 120MB (int8)
  if (1LL * new_w * new_h > max_cells) {
    ROS_ERROR("Merged map expansion too large (%d x %d). Refuse expand.", new_w, new_h);
    return;
  }

  nav_msgs::OccupancyGrid new_map;
  new_map.header.frame_id = merged_map.header.frame_id;
  new_map.header.stamp = merged_map.header.stamp;
  new_map.info = merged_map.info;
  new_map.info.origin.position.x = new_minx;
  new_map.info.origin.position.y = new_miny;
  new_map.info.width  = new_w;
  new_map.info.height = new_h;
  new_map.data.assign(new_w * new_h, -1);

  // 旧图拷贝到新图：计算旧 origin 在新图坐标里的偏移（单位：cell）
  const int off_x = (int)std::lround((cur_minx - new_minx) / res);
  const int off_y = (int)std::lround((cur_miny - new_miny) / res);

  const int old_w = (int)merged_map.info.width;
  const int old_h = (int)merged_map.info.height;

  for (int y = 0; y < old_h; ++y) {
    for (int x = 0; x < old_w; ++x) {
      const int old_idx = x + y * old_w;
      const int8_t v = merged_map.data[old_idx];
      if (v == -1) continue;

      const int nx = x + off_x;
      const int ny = y + off_y;
      if (nx < 0 || nx >= new_w || ny < 0 || ny >= new_h) continue;

      new_map.data[nx + ny * new_w] = v;
    }
  }

  merged_map = std::move(new_map);

  ROS_WARN_THROTTLE(1.0, "Expanded merged_map to [%d x %d], origin=(%.2f, %.2f)",
                    (int)merged_map.info.width, (int)merged_map.info.height,
                    merged_map.info.origin.position.x, merged_map.info.origin.position.y);
}

// ---------------- callbacks ----------------
static void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg, int rid)
{
  // rid: 1..n_robot
  if (rid < 1 || rid > n_robot) return;
  robot_maps[rid - 1] = *msg;
  robot_received[rid - 1] = true;
}

// ---------------- merge logic ----------------
static void mergeOneRobotMap(const nav_msgs::OccupancyGrid& tmp_map,
                             const geometry_msgs::TransformStamped& transform_map,
                             int r_index_x, int r_index_y)
{
  const double merge_range = 10.0;
  const int step = (int)(merge_range / tmp_map.info.resolution);

  // ====== 先根据 merge window 估计会落到全局的 world bbox，扩一次 merged_map ======
  // 注意：你当前 merge 使用的是“仅平移”，所以 bbox 也按平移估计
  const int i0 = std::max(0, r_index_x - step);
  const int i1 = std::min((int)tmp_map.info.width  - 1, r_index_x + step);
  const int j0 = std::max(0, r_index_y - step);
  const int j1 = std::min((int)tmp_map.info.height - 1, r_index_y + step);

  // window 四角点（local map坐标）
  auto localCellToWorld = [&](int i, int j, double& gx, double& gy){
    const double lx = DISCXY2CONT(i, tmp_map.info.resolution) + tmp_map.info.origin.position.x;
    const double ly = DISCXY2CONT(j, tmp_map.info.resolution) + tmp_map.info.origin.position.y;
    // 仍按你当前逻辑：只用平移
    gx = lx + transform_map.transform.translation.x;
    gy = ly + transform_map.transform.translation.y;
  };

  double gx00, gy00, gx01, gy01, gx10, gy10, gx11, gy11;
  localCellToWorld(i0, j0, gx00, gy00);
  localCellToWorld(i0, j1, gx01, gy01);
  localCellToWorld(i1, j0, gx10, gy10);
  localCellToWorld(i1, j1, gx11, gy11);

  const double minx = std::min(std::min(gx00, gx01), std::min(gx10, gx11));
  const double maxx = std::max(std::max(gx00, gx01), std::max(gx10, gx11));
  const double miny = std::min(std::min(gy00, gy01), std::min(gy10, gy11));
  const double maxy = std::max(std::max(gy00, gy01), std::max(gy10, gy11));

  ensureMergedCovers(minx, miny, maxx, maxy, /*margin=*/5.0);

  // ====== 正式 merge（你原来的逻辑 + 越界保护保留） ======
  for (int i = r_index_x - step; i <= r_index_x + step; i++)
  {
    for (int j = r_index_y - step; j <= r_index_y + step; j++)
    {
      if (i < 0 || i >= (int)tmp_map.info.width || j < 0 || j >= (int)tmp_map.info.height) continue;

      int8_t v = tmp_map.data[i + j * tmp_map.info.width];
      if (v == -1) continue;

      double gx = DISCXY2CONT(i, tmp_map.info.resolution) + tmp_map.info.origin.position.x
                  + transform_map.transform.translation.x;
      double gy = DISCXY2CONT(j, tmp_map.info.resolution) + tmp_map.info.origin.position.y
                  + transform_map.transform.translation.y;

      int gix = CONTXY2DISC(gx - merged_map.info.origin.position.x, merged_map.info.resolution);
      int giy = CONTXY2DISC(gy - merged_map.info.origin.position.y, merged_map.info.resolution);

      if (!insideMerged(gix, giy)) continue;

      int idx = gix + giy * merged_map.info.width;

      if (merged_map.data[idx] == -1)
      {
        merged_map.data[idx] = v;
      }
      else
      {
        int m = merged_map.data[idx];
        int merged_val = int(0.2 * double(m)) + int(0.8 * double(v));
        merged_map.data[idx] = (merged_val >= 80) ? 100 : 0;
      }
    }
  }
}

static void addAndDeleRobotObs(const tf::TransformListener& listener, nav_msgs::OccupancyGrid& planning_map)
{
  for (int k = 1; k <= n_robot; k++)
  {
    geometry_msgs::TransformStamped transform_rp;
    if (!findTF("map", "robot_" + to_string(k) + "/base_link", transform_rp, listener))
      continue;

    int rx = CONTXY2DISC(transform_rp.transform.translation.x - merged_map.info.origin.position.x, merged_map.info.resolution);
    int ry = CONTXY2DISC(transform_rp.transform.translation.y - merged_map.info.origin.position.y, merged_map.info.resolution);

    for (int i = rx - 2; i <= rx + 2; i++)
    {
      for (int j = ry - 2; j <= ry + 2; j++)
      {
        if (i < 0 || i >= (int)merged_map.info.width || j < 0 || j >= (int)merged_map.info.height) continue;
        planning_map.data[i + j * merged_map.info.width] = (k == robot_id) ? 0 : 100;
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ✅ 用 ~ 读参数（别用 this_node::getName 拼字符串）
  pnh.param("n_robot", n_robot, 3);
  pnh.param("robot_id", robot_id, 1);
  pnh.param("map_x", map_x, -20.0);
  pnh.param("map_y", map_y, -20.0);
  pnh.param("map_width", map_width, 400);
  pnh.param("map_height", map_height, 400);

  robot_maps.resize(n_robot);
  robot_received.assign(n_robot, false);

  tf::TransformListener listener;

  vector<ros::Subscriber> map_subs;
  map_subs.reserve(n_robot);
  for (int i = 1; i <= n_robot; i++)
  {
    string topic = "/robot_" + to_string(i) + "/map";
    map_subs.push_back(nh.subscribe<nav_msgs::OccupancyGrid>(topic, 2,
                      boost::bind(mapCallBack, _1, i)));
    ROS_INFO("Subscribe: %s", topic.c_str());
  }

  ros::Publisher merged_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  ros::Publisher merged_planning_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("planning_map", 1);

  ros::Rate rate(5);
  initMergedMap();

  while (ros::ok())
  {
    ros::spinOnce();

    if (!merge_init) { rate.sleep(); continue; }

    // 每轮：把三张 map 都 merge 一遍（谁收到过就 merge 谁）
    bool any = false;
    for (int rid = 1; rid <= n_robot; rid++)
    {
      if (!robot_received[rid - 1]) continue;

      const auto& tmp_map = robot_maps[rid - 1];
      // 该机器人 map frame（你原来是 robot_i/map）
      string r_map_frame_id = "robot_" + to_string(rid) + "/map";
      geometry_msgs::TransformStamped transform_map, transform_robot;

      // 先拿 map -> robot_i/map
      if (!findTF("map", r_map_frame_id, transform_map, listener)) continue;
      // 再拿 robot_i/map -> robot_i/base_link
      if (!findTF(r_map_frame_id, "robot_" + to_string(rid) + "/base_link", transform_robot, listener)) continue;

      int r_index_x = CONTXY2DISC(transform_robot.transform.translation.x - tmp_map.info.origin.position.x, tmp_map.info.resolution);
      int r_index_y = CONTXY2DISC(transform_robot.transform.translation.y - tmp_map.info.origin.position.y, tmp_map.info.resolution);

      mergeOneRobotMap(tmp_map, transform_map, r_index_x, r_index_y);

      merged_map.header.stamp = ros::Time::now();
      any = true;
    }

    if (any)
    {
      nav_msgs::OccupancyGrid planning_map = merged_map;
      addAndDeleRobotObs(listener, planning_map);

      // ✅ 建议只跑一个 merge_map 节点发布 /map；如果你仍然每个机器人都起这个节点，会抢 /map
      merged_map_pub.publish(merged_map);
      merged_planning_map_pub.publish(planning_map);
    }

    rate.sleep();
  }
  return 0;
}