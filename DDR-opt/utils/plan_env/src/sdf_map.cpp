#include "plan_env/sdf_map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include <cmath>
#include <limits>
#include <algorithm>

// void SDFmap::odomCallback(const carstatemsgs::CarState::ConstPtr &msg){
//   odom_ = *msg;
//   has_odom_ = true;
// }

void SDFmap::occGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(occ_grid_mutex_);
  latest_occ_grid_ = *msg;
  has_occ_grid_ = true;
}

void SDFmap::publishFusedCloud(){
  if (cloud_.points.empty() && !has_occ_grid_) return;

  // 这里不污染 member cloud_，只做临时可视化融合
  pcl::PointCloud<pcl::PointXYZ> fused_cloud = cloud_;
  appendLocalOccupiedCellsFromGrid(fused_cloud);

  fused_cloud.width = fused_cloud.points.size();
  fused_cloud.height = 1;
  fused_cloud.is_dense = true;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(fused_cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = ros::Time::now();
  pub_fused_cloud_.publish(cloud_msg);
}

void SDFmap::appendLocalOccupiedCellsFromGrid(pcl::PointCloud<pcl::PointXYZ>& cloud) {
  if (!fuse_occ_grid_to_cloud_) return;

  nav_msgs::OccupancyGrid map_msg;
  {
    std::lock_guard<std::mutex> lock(occ_grid_mutex_);
    if (!has_occ_grid_) return;
    map_msg = latest_occ_grid_;
  }

  if (map_msg.info.width == 0 || map_msg.info.height == 0) return;
  if (map_msg.data.empty()) return;

  if (map_msg.header.frame_id != "map") {
    ROS_WARN_THROTTLE(1.0, "[SDFmap] OccupancyGrid frame is %s, expected map. Skip fusion.",
                      map_msg.header.frame_id.c_str());
    return;
  }

  const double res = map_msg.info.resolution;
  const int width  = static_cast<int>(map_msg.info.width);
  const int height = static_cast<int>(map_msg.info.height);

  const double origin_x = map_msg.info.origin.position.x;
  const double origin_y = map_msg.info.origin.position.y;

  const double half_x = 0.5 * local_map_size_x_;
  const double half_y = 0.5 * local_map_size_y_;

  const double x_min = odom_pos_.x() - half_x;
  const double x_max = odom_pos_.x() + half_x;
  const double y_min = odom_pos_.y() - half_y;
  const double y_max = odom_pos_.y() + half_y;

  int ix_min = std::max(0, static_cast<int>(std::floor((x_min - origin_x) / res)));
  int ix_max = std::min(width - 1, static_cast<int>(std::floor((x_max - origin_x) / res)));
  int iy_min = std::max(0, static_cast<int>(std::floor((y_min - origin_y) / res)));
  int iy_max = std::min(height - 1, static_cast<int>(std::floor((y_max - origin_y) / res)));

  if (ix_min > ix_max || iy_min > iy_max) return;

  const int approx_cnt =
      ((ix_max - ix_min + occ_grid_stride_) / occ_grid_stride_) *
      ((iy_max - iy_min + occ_grid_stride_) / occ_grid_stride_);
  if (approx_cnt > 0) {
    cloud.points.reserve(cloud.points.size() + approx_cnt / 4);
  }

  for (int ix = ix_min; ix <= ix_max; ix += occ_grid_stride_) {
    for (int iy = iy_min; iy <= iy_max; iy += occ_grid_stride_) {
      const int idx = iy * width + ix;
      if (idx < 0 || idx >= static_cast<int>(map_msg.data.size())) continue;

      const int8_t occ = map_msg.data[idx];
      if (occ < occ_grid_threshold_) continue;

      pcl::PointXYZ pt;
      pt.x = origin_x + (static_cast<double>(ix) + 0.5) * res;
      pt.y = origin_y + (static_cast<double>(iy) + 0.5) * res;
      pt.z = 0.285;   // 仅用于 fused cloud 可视化
      cloud.points.push_back(pt);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
}

void SDFmap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
  static tf2_ros::Buffer tf_buffer(ros::Duration(10.0));
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped sensor_to_map;
  geometry_msgs::TransformStamped base_to_map;

  // 1) 用传感器 frame 做点云坐标变换
  try {
    sensor_to_map = tf_buffer.lookupTransform("map", msg->header.frame_id,
                                              msg->header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // 2) 用 base_link 做机器人中心位姿；若失败则退化为传感器位姿
  try {
    base_to_map = tf_buffer.lookupTransform("map", robot_ns_+"/base_link",
                                            msg->header.stamp, ros::Duration(0.05));
  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[SDFmap] map->base_link TF unavailable, fallback to sensor frame: %s", ex.what());
    base_to_map = sensor_to_map;
  }

  odom_pos_.head(2) = Eigen::Vector2d(base_to_map.transform.translation.x,
                                      base_to_map.transform.translation.y);
  odom_pos_[2] = tf::getYaw(base_to_map.transform.rotation);

  // Perform coordinate transformation for sensor cloud
  sensor_msgs::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, sensor_to_map);

  // Convert to PCL format: cloud_ 只保留原始实时点云
  pcl::fromROSMsg(transformed_cloud, cloud_);

  // 统一整理成非组织点云，避免后面可视化/转换断言
  cloud_.width = cloud_.points.size();
  cloud_.height = 1;
  cloud_.is_dense = true;

  // Publish merged cloud for visualization in RViz (raw sensor + map occupied)
  publishFusedCloud();

  occ_need_update_ = true;
}

void SDFmap::updateOccupancyCallback(const ros::TimerEvent& /*event*/){
  if(!local_map_inited_) return;

  // 当前局部窗口边界（以机器人中心为中心）
  const double half_x = 0.5 * local_map_size_x_;
  const double half_y = 0.5 * local_map_size_y_;

  global_x_lower_ = odom_pos_.x() - half_x;
  global_x_upper_ = odom_pos_.x() + half_x;
  global_y_lower_ = odom_pos_.y() - half_y;
  global_y_upper_ = odom_pos_.y() + half_y;

  inv_grid_interval_ = 1.0 / grid_interval_;

  if(!occ_need_update_) return;

  // rolling local window：每次重建当前局部窗口
  std::fill(gridmap_, gridmap_ + GLXY_SIZE_, Unknown);
  std::fill(occupancy_map_.begin(), occupancy_map_.end(), clamp_min_log_);
  std::fill(count_hit_.begin(), count_hit_.end(), 0);
  std::fill(count_hit_and_miss_.begin(), count_hit_and_miss_.end(), 0);

  if(!if_perspective_){
    const double rx = std::ceil(detection_range_ / grid_interval_) * grid_interval_;
    const double ry = rx;

    x_lower_ = std::max(odom_pos_.x() - rx, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + rx, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - ry, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + ry, global_y_upper_);

    X_SIZE_ = GLX_SIZE_;
    Y_SIZE_ = GLY_SIZE_;
    XY_SIZE_ = GLXY_SIZE_;

    // 1) 只用真实传感器点云做 raycast
    raycastProcess();

    if(if_cirSupRaycast_){
      static int cirSup = 1;
      cirSup ++;
      if(cirSup % 3 == 0){
        cirSupRaycastProcess();
        cirSup = 1;
      }
    }

    // 2) 直接把 /map occupied cells 作为先验障碍灌入局部 occupancy/gridmap
    if (fuse_occ_grid_to_cloud_) {
      nav_msgs::OccupancyGrid map_msg;
      bool map_ok = false;
      {
        std::lock_guard<std::mutex> lock(occ_grid_mutex_);
        if (has_occ_grid_) {
          map_msg = latest_occ_grid_;
          map_ok = true;
        }
      }

      if (map_ok && map_msg.info.width > 0 && map_msg.info.height > 0 &&
          !map_msg.data.empty() && map_msg.header.frame_id == "map") {

        const double res = map_msg.info.resolution;
        const int map_w = static_cast<int>(map_msg.info.width);
        const int map_h = static_cast<int>(map_msg.info.height);
        const double origin_x = map_msg.info.origin.position.x;
        const double origin_y = map_msg.info.origin.position.y;

        int ix_min = std::max(0, static_cast<int>(std::floor((x_lower_ - origin_x) / res)));
        int ix_max = std::min(map_w - 1, static_cast<int>(std::floor((x_upper_ - origin_x) / res)));
        int iy_min = std::max(0, static_cast<int>(std::floor((y_lower_ - origin_y) / res)));
        int iy_max = std::min(map_h - 1, static_cast<int>(std::floor((y_upper_ - origin_y) / res)));

        if (!(ix_min > ix_max || iy_min > iy_max)) {
          for (int ix = ix_min; ix <= ix_max; ix += occ_grid_stride_) {
            for (int iy = iy_min; iy <= iy_max; iy += occ_grid_stride_) {
              const int map_idx = iy * map_w + ix;
              if (map_idx < 0 || map_idx >= static_cast<int>(map_msg.data.size())) continue;

              const int8_t occ = map_msg.data[map_idx];
              if (occ < occ_grid_threshold_) continue;

              const double wx = origin_x + (static_cast<double>(ix) + 0.5) * res;
              const double wy = origin_y + (static_cast<double>(iy) + 0.5) * res;
              const Eigen::Vector2d coord(wx, wy);

              if (!isInGloMap(coord)) continue;

              const Eigen::Vector2i idx = coord2gridIndex(coord);
              const int vecIndex = Index2Vectornum(idx);

              occupancy_map_[vecIndex] = clamp_max_log_;
              gridmap_[vecIndex] = Occupied;
            }
          }
        }
      }
    }

    RemoveOutliers();

    Eigen::Vector2i min_id(0, 0);
    Eigen::Vector2i max_id(GLX_SIZE_ - 1, GLY_SIZE_ - 1);
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    // 根据 occupancy_map_ 生成最终 gridmap_
    for (int x = min_id.x(); x <= max_id.x(); x++) {
      for (int y = min_id.y(); y <= max_id.y(); y++) {
        int vecIndex = Index2Vectornum(x, y);

        if (occupancy_map_[vecIndex] > min_occupancy_log_) {
          gridmap_[vecIndex] = Occupied;
        } else if (gridmap_[vecIndex] == Unknown &&
                   occupancy_map_[vecIndex] >= clamp_min_log_ &&
                   occupancy_map_[vecIndex] <= min_occupancy_log_) {
          gridmap_[vecIndex] = Unoccupied;
        }
      }
    }
  }
  else{
    x_lower_ = std::max(odom_pos_.x() - std::ceil(detection_range_ / grid_interval_) * grid_interval_, global_x_lower_);
    x_upper_ = std::min(odom_pos_.x() + std::ceil(detection_range_ / grid_interval_) * grid_interval_, global_x_upper_);
    y_lower_ = std::max(odom_pos_.y() - std::ceil(detection_range_ / grid_interval_) * grid_interval_, global_y_lower_);
    y_upper_ = std::min(odom_pos_.y() + std::ceil(detection_range_ / grid_interval_) * grid_interval_, global_y_upper_);

    X_SIZE_ = std::ceil((x_upper_ - x_lower_) / grid_interval_);
    Y_SIZE_ = std::ceil((y_upper_ - y_lower_) / grid_interval_);
    XY_SIZE_ = X_SIZE_ * Y_SIZE_;

    Eigen::Vector2i min_id, max_id;
    min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
    max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

    for (int x = min_id.x(); x <= max_id.x(); x++) {
      for (int y = min_id.y(); y <= max_id.y(); y++) {
        if(gridmap_[Index2Vectornum(x,y)] == Unknown)
          gridmap_[Index2Vectornum(x,y)] = Unoccupied;
      }
    }

    // 真实点云写入障碍
    for(const auto& point : cloud_.points){
      Eigen::Vector2d coord(point.x, point.y);
      if(!isInGloMap(coord)){
        continue;
      }
      Eigen::Vector2i idx = coord2gridIndex(coord);
      gridmap_[Index2Vectornum(idx)] = Occupied;
    }

    // /map occupied 直接写入障碍
    if (fuse_occ_grid_to_cloud_) {
      nav_msgs::OccupancyGrid map_msg;
      bool map_ok = false;
      {
        std::lock_guard<std::mutex> lock(occ_grid_mutex_);
        if (has_occ_grid_) {
          map_msg = latest_occ_grid_;
          map_ok = true;
        }
      }

      if (map_ok && map_msg.info.width > 0 && map_msg.info.height > 0 &&
          !map_msg.data.empty() && map_msg.header.frame_id == "map") {

        const double res = map_msg.info.resolution;
        const int map_w = static_cast<int>(map_msg.info.width);
        const int map_h = static_cast<int>(map_msg.info.height);
        const double origin_x = map_msg.info.origin.position.x;
        const double origin_y = map_msg.info.origin.position.y;

        int ix_min = std::max(0, static_cast<int>(std::floor((x_lower_ - origin_x) / res)));
        int ix_max = std::min(map_w - 1, static_cast<int>(std::floor((x_upper_ - origin_x) / res)));
        int iy_min = std::max(0, static_cast<int>(std::floor((y_lower_ - origin_y) / res)));
        int iy_max = std::min(map_h - 1, static_cast<int>(std::floor((y_upper_ - origin_y) / res)));

        if (!(ix_min > ix_max || iy_min > iy_max)) {
          for (int ix = ix_min; ix <= ix_max; ix += occ_grid_stride_) {
            for (int iy = iy_min; iy <= iy_max; iy += occ_grid_stride_) {
              const int map_idx = iy * map_w + ix;
              if (map_idx < 0 || map_idx >= static_cast<int>(map_msg.data.size())) continue;

              const int8_t occ = map_msg.data[map_idx];
              if (occ < occ_grid_threshold_) continue;

              const double wx = origin_x + (static_cast<double>(ix) + 0.5) * res;
              const double wy = origin_y + (static_cast<double>(iy) + 0.5) * res;
              const Eigen::Vector2d coord(wx, wy);

              if (!isInGloMap(coord)) continue;

              const Eigen::Vector2i idx = coord2gridIndex(coord);
              gridmap_[Index2Vectornum(idx)] = Occupied;
            }
          }
        }
      }
    }
  }

  has_map_ = true;
  esdf_need_update_ = true;
  occ_need_update_ = false;
}

void SDFmap::raycastProcess(){
  int points_cnt = cloud_.points.size();

  update_odom_ = odom_pos_.head(2);
  Eigen::Vector2d cur_point;
  int vox_idx;
  double length;

  for (int i = 0; i < points_cnt; ++i) {
    cur_point << cloud_.points[i].x, cloud_.points[i].y;

    if(!isInGloMap(cur_point)){
      cur_point = closetPointInMap(cur_point, update_odom_);
      length = (cur_point - update_odom_).norm();
      if(length > detection_range_){
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
      }
      vox_idx = setCacheOccupancy(cur_point, 0);
    }
    else {
      length = (cur_point - update_odom_).norm();
      if (length > detection_range_) {
        cur_point = (cur_point - update_odom_) / length * detection_range_ + update_odom_;
        vox_idx = setCacheOccupancy(cur_point, 0);
      } else {
        vox_idx = setCacheOccupancy(cur_point, 1);
      }
    }

    std::vector<Eigen::Vector2i> line =
        getGridsBetweenPoints2D(coord2gridIndex(update_odom_), coord2gridIndex(cur_point));

    int size = line.size() - 1;
    for (int i = 0; i < size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }
  }

  updateOccupancyMap();
}

// Add a circle to prevent no points, resulting in no safe position points
void SDFmap::cirSupRaycastProcess(){
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
  double length;
  int vox_idx;

  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_ + 1e-10;  x += 2 * detection_range_){
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_ + 1e-10;  y += grid_interval_){
      cir_points.emplace_back(x, y);
    }
  }
  for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_ + 1e-10;  y += 2 * detection_range_){
    for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_ + 1e-10;  x += grid_interval_){
      cir_points.emplace_back(x, y);
    }
  }

  RayCaster raycaster;

  for(auto cir_point : cir_points){

    if(hrz_limited_){
      double angle = atan2(cir_point.y() - odom_pos_.y(), cir_point.x() - odom_pos_.x());
      angle = normalize_angle(angle - odom_pos_.z());
      if(angle < -hrz_laser_range_dgr_ / 2.2 || angle > hrz_laser_range_dgr_ / 2.2){
        continue;
      }
    }

    if(!isInGloMap(cir_point)){
      cir_point = closetPointInMap(cir_point, update_odom_);
    }

    length = (cir_point - update_odom_).norm();
    if(length > detection_range_){
      cir_point = (cir_point - update_odom_) / length * detection_range_ + update_odom_;
    }

    std::vector<Eigen::Vector2i> line;
    raycaster.setInput(Eigen::Vector3d(cir_point.x(), cir_point.y(), 0.1) / grid_interval_,
                       Eigen::Vector3d(update_odom_.x(), update_odom_.y(), 0.1) / grid_interval_);

    bool occ = false;
    Eigen::Vector3d ray_pt;
    while (raycaster.step(ray_pt)) {
      Eigen::Vector2d tmp = (ray_pt.head(2) + half) * grid_interval_;
      Eigen::Vector2i tmp_idx = coord2gridIndex(tmp);
      line.emplace_back(tmp_idx);
      const Eigen::Vector2i g = coord2gridIndex(tmp);
      const int c = Index2Vectornum(g);

      if (gridmap_[c] == Occupied ||
          (g.y() < GLY_SIZE_ - 1 && gridmap_[c + 1] == Occupied) ||
          (g.y() > 0            && gridmap_[c - 1] == Occupied) ||
          (g.x() < GLX_SIZE_ - 1 && gridmap_[c + GLY_SIZE_] == Occupied) ||
          (g.x() > 0             && gridmap_[c - GLY_SIZE_] == Occupied)) {
        occ = true;
        break;
      }
    }
    if(occ) continue;

    int size = line.size() - 1;
    for (int i = 0; i < size; i++) {
      vox_idx = setCacheOccupancy(line[i], 0);
    }
  }

  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 4 * count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    log_odds_update = 0.0;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
                    idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                 clamp_max_log_);
  }
}

void SDFmap::updateOccupancyMap(){
  Eigen::Vector2i min_id, max_id;
  min_id = coord2gridIndex(Eigen::Vector2d(x_lower_, y_lower_));
  max_id = coord2gridIndex(Eigen::Vector2d(x_upper_, y_upper_));

  while (!cache_voxel_.empty()) {
    Eigen::Vector2i idx = cache_voxel_.front();
    int idx_ctns = Index2Vectornum(idx);
    cache_voxel_.pop();

    double log_odds_update =
      count_hit_[idx_ctns] >= count_hit_and_miss_[idx_ctns] - 3 * count_hit_[idx_ctns] ?
      prob_hit_log_ :
      prob_miss_log_;

    count_hit_[idx_ctns] = count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && occupancy_map_[idx_ctns] >= clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && occupancy_map_[idx_ctns] <= clamp_min_log_) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
                    idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      occupancy_map_[idx_ctns] = clamp_min_log_;
    }

    occupancy_map_[idx_ctns] =
        std::min(std::max(occupancy_map_[idx_ctns] + log_odds_update, clamp_min_log_),
                 clamp_max_log_);
  }
}

void SDFmap::RemoveOutliers(){
  std::vector<Eigen::Vector2d> cir_points;
  for(double x = odom_pos_.x() - detection_range_; x < odom_pos_.x() + detection_range_ + 1e-10;  x += grid_interval_){
    for(double y = odom_pos_.y() - detection_range_; y < odom_pos_.y() + detection_range_ + 1e-10;  y += grid_interval_){
      cir_points.emplace_back(x, y);
    }
  }

  double xlow = global_x_lower_ + grid_interval_;
  double xup = global_x_upper_ - grid_interval_;
  double ylow = global_y_lower_ + grid_interval_;
  double yup = global_y_upper_ - grid_interval_;

  for(auto cir_point : cir_points){
    if(cir_point.x() > xlow && cir_point.x() < xup && cir_point.y() > ylow && cir_point.y() < yup){
      if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] == Unknown){
        if(gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) + 1] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) - 1] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) + GLY_SIZE_] == Unoccupied &&
           gridmap_[Index2Vectornum(coord2gridIndex(cir_point)) - GLY_SIZE_] == Unoccupied){
          gridmap_[Index2Vectornum(coord2gridIndex(cir_point))] = Unoccupied;
        }
      }
    }
  }

  Eigen::Vector2i idx = coord2gridIndex(Eigen::Vector2d(odom_pos_.x(), odom_pos_.y()));
  for(int i = -1; i <= 1; i++){
    for(int j = -1; j <= 1; j++){
      if(gridmap_[Index2Vectornum(idx.x() + i, idx.y() + j)] == Unknown){
        gridmap_[Index2Vectornum(idx.x() + i, idx.y() + j)] = Unoccupied;
      }
    }
  }
}

int SDFmap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
  if (occ != 1 && occ != 0) return -1;

  Eigen::Vector2i idx = coord2gridIndex(pos);
  if(idx.x() < 0 || idx.x() >= GLX_SIZE_ || idx.y() < 0 || idx.y() >= GLY_SIZE_) return -1;
  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

int SDFmap::setCacheOccupancy(Eigen::Vector2i idx, int occ) {
  if (occ != 1 && occ != 0) return -1;
  if(idx.x() < 0 || idx.x() >= GLX_SIZE_ || idx.y() < 0 || idx.y() >= GLY_SIZE_) return -1;
  int idx_ctns = Index2Vectornum(idx);

  count_hit_and_miss_[idx_ctns] += 1;

  if (count_hit_and_miss_[idx_ctns] == 1) {
    cache_voxel_.push(idx);
  }

  if (occ == 1) count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

std::vector<Eigen::Vector2i> SDFmap::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
  std::vector<Eigen::Vector2i> line;

  int dx = abs(end.x() - start.x());
  int dy = abs(end.y() - start.y());
  int sx = (start.x() < end.x()) ? 1 : -1;
  int sy = (start.y() < end.y()) ? 1 : -1;
  int err = dx - dy;

  double x0 = start.x();
  double y0 = start.y();

  while (true) {
    line.emplace_back(x0, y0);
    if (x0 == end.x() && y0 == end.y()) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return line;
}

void SDFmap::updateESDFCallback(const ros::TimerEvent& /*event*/){
  if(!esdf_need_update_) return;
  updateESDF2d();

  has_esdf_ = true;
  esdf_need_update_ = false;
}

void SDFmap::visCallback(const ros::TimerEvent& /*event*/){
  if(has_map_){
    publish_gridmap();
  }
  if(has_esdf_){
    publish_ESDF();
    publish_Neighboring_PC();
    // publish_ESDFGrad();
  }
}

inline void SDFmap::grid_insertbox(Eigen::Vector3d location, Eigen::Matrix3d euler, Eigen::Vector3d size){
  Eigen::Vector3d x(1,0,0);
  Eigen::Vector3d y(0,1,0);
  Eigen::Vector3d z(0,0,1);
  x  = euler * x;
  y  = euler * y;
  z  = euler * z;

  float insert_interval = 0.5;
  for(float i = -size.x() / 2; i <= size.x() / 2; i += grid_interval_ * insert_interval)
    for(float j = -size.y() / 2; j <= size.y() / 2; j += grid_interval_ * insert_interval)
      for(float k = -size.z() / 2; k <= size.z() / 2; k += grid_interval_ * insert_interval){
        Eigen::Vector3d point = location + i * x + j * y + k * z;
        setObs(point);
      }
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const Eigen::Vector2i &index){
  Eigen::Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)index(1) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2d SDFmap::gridIndex2coordd(const int &x, const int &y){
  Eigen::Vector2d pt;
  pt(0) = ((double)x + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = ((double)y + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

Eigen::Vector2i SDFmap::coord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_), 0), GLX_SIZE_ - 1),
         std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_), 0), GLY_SIZE_ - 1);
  return idx;
}

void SDFmap::setObs(const Eigen::Vector3d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_)
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

void SDFmap::setObs(const Eigen::Vector2d coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  if (coord_x < global_x_lower_ || coord_y < global_y_lower_ ||
      coord_x >= global_x_upper_ || coord_y >= global_y_upper_)
    return;
  int idx_x = static_cast<int>((coord_x - global_x_lower_) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - global_y_lower_) * inv_grid_interval_);
  gridmap_[idx_x * GLY_SIZE_ + idx_y] = Occupied;
}

Eigen::Vector2i SDFmap::vectornum2gridIndex(const int &num){
  Eigen::Vector2i index;
  index(0) = num / GLY_SIZE_;
  index(1) = num % GLY_SIZE_;
  return index;
}

int SDFmap::Index2Vectornum(const int &x, const int &y){
  return x * GLY_SIZE_ + y;
}

int SDFmap::Index2Vectornum(const Eigen::Vector2i &index){
  return index.x() * GLY_SIZE_ + index.y();
}

void SDFmap::publish_gridmap(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;

  for(int idx = 1; idx < GLXY_SIZE_; idx++){
    if(gridmap_[idx] == Occupied){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      pt.intensity = 0.0;
      cloud_vis.points.push_back(pt);
    }
    if(gridmap_[idx] == Unknown){
      Eigen::Vector2d corrd = gridIndex2coordd(vectornum2gridIndex(idx));
      pcl::PointXYZI pt;
      pt.x = corrd.x(); pt.y = corrd.y(); pt.z = 0.1;
      pt.intensity = 8.0;
      cloud_vis.points.push_back(pt);
    }
  }

  pcl::PointXYZI pt;
  pt.x = 100.0; pt.y = 100.0; pt.z = 0.1;
  pt.intensity = 10.0;
  cloud_vis.points.push_back(pt);

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "map";
  pub_gridmap_.publish(map_vis);
}

uint8_t SDFmap::CheckCollisionBycoord(const Eigen::Vector2d &pt){
  if(pt.x() > global_x_upper_ || pt.x() < global_x_lower_ || pt.y() > global_y_upper_ || pt.y() < global_y_lower_){
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

uint8_t SDFmap::CheckCollisionBycoord(const double ptx, const double pty){
  if(ptx > global_x_upper_ || ptx < global_x_lower_ || pty > global_y_upper_ || pty < global_y_lower_){
    return Unknown;
  }
  Eigen::Vector2i index = coord2gridIndex(Eigen::Vector2d(ptx, pty));
  return gridmap_[index.x() * GLY_SIZE_ + index.y()];
}

bool SDFmap::isInGloMap(const Eigen::Vector2d &pt){
  return pt.x() < global_x_upper_ && pt.x() > global_x_lower_ &&
         pt.y() < global_y_upper_ && pt.y() > global_y_lower_;
}

Eigen::Vector2d SDFmap::closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos){
  Eigen::Vector2d diff = pt - pos;
  Eigen::Vector2d max_tc = Eigen::Vector2d(global_x_upper_, global_y_upper_) - pos;
  Eigen::Vector2d min_tc = Eigen::Vector2d(global_x_lower_, global_y_lower_) - pos;

  double min_t = 1000000;

  for (int i = 0; i < 2; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return pos + (min_t - 1e-3) * diff;
}

void SDFmap::updateESDF2d(){
  Eigen::Vector2i min_esdf(
      floor(std::max(0.0, odom_pos_.x() - detection_range_ - global_x_lower_) * inv_grid_interval_),
      floor(std::max(0.0, odom_pos_.y() - detection_range_ - global_y_lower_) * inv_grid_interval_));

  Eigen::Vector2i max_esdf(
      ceil(std::min(global_x_upper_ - global_x_lower_, odom_pos_.x() + detection_range_ - global_x_lower_) * inv_grid_interval_) - 1,
      ceil(std::min(global_y_upper_ - global_y_lower_, odom_pos_.y() + detection_range_ - global_y_lower_) * inv_grid_interval_) - 1);

  min_esdf.x() = std::max(0, std::min(min_esdf.x(), GLX_SIZE_ - 1));
  min_esdf.y() = std::max(0, std::min(min_esdf.y(), GLY_SIZE_ - 1));
  max_esdf.x() = std::max(0, std::min(max_esdf.x(), GLX_SIZE_ - 1));
  max_esdf.y() = std::max(0, std::min(max_esdf.y(), GLY_SIZE_ - 1));

  const int update_X_SIZE = max_esdf.x() - min_esdf.x();
  const int update_Y_SIZE = max_esdf.y() - min_esdf.y();

  if (update_X_SIZE <= 0 || update_Y_SIZE <= 0) return;

  const int X = update_X_SIZE + 1;
  const int Y = update_Y_SIZE + 1;
  const int update_XY_SIZE = X * Y;

  std::vector<double> tmp_buffer1(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer(update_XY_SIZE, 0.0);
  std::vector<double> distance_buffer_neg(update_XY_SIZE, 0.0);

  // positive DT
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
      [&](int y) {
        const int gx = x + min_esdf.x();
        const int gy = y + min_esdf.y();
        return (gridmap_[gx * GLY_SIZE_ + gy] == Occupied) ? 0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) {
        tmp_buffer1[x * Y + y] = val;
      },
      0, update_Y_SIZE, Y);
  }

  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF(
      [&](int x) { return tmp_buffer1[x * Y + y]; },
      [&](int x, double val) { distance_buffer[x * Y + y] = grid_interval_ * std::sqrt(val); },
      0, update_X_SIZE, X);
  }

  // negative DT
  for (int x = 0; x <= update_X_SIZE; x++) {
    fillESDF(
      [&](int y) {
        const int gx = x + min_esdf.x();
        const int gy = y + min_esdf.y();
        const int state = gridmap_[gx * GLY_SIZE_ + gy];
        return (state == Unoccupied || state == Unknown) ? 0.0 : std::numeric_limits<double>::max();
      },
      [&](int y, double val) {
        tmp_buffer1[x * Y + y] = val;
      },
      0, update_Y_SIZE, Y);
  }

  for (int y = 0; y <= update_Y_SIZE; y++) {
    fillESDF(
      [&](int x) { return tmp_buffer1[x * Y + y]; },
      [&](int x, double val) { distance_buffer_neg[x * Y + y] = grid_interval_ * std::sqrt(val); },
      0, update_X_SIZE, X);
  }

  // combine
  for (int x = 0; x <= update_X_SIZE; x++) {
    for (int y = 0; y <= update_Y_SIZE; y++) {
      const int global_idx = (x + min_esdf.x()) * GLY_SIZE_ + (y + min_esdf.y());
      const int local_idx  = x * Y + y;

      distance_buffer_all_[global_idx] = distance_buffer[local_idx];
      if (distance_buffer_neg[local_idx] > 0.0) {
        distance_buffer_all_[global_idx] += (-distance_buffer_neg[local_idx] + grid_interval_);
      }
    }
  }
}

template <typename F_get_val, typename F_set_val>
void SDFmap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size) {
  int v[dim_size];
  double z[dim_size + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFmap::publish_ESDF(){
  pcl::PointCloud<pcl::PointXYZI> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  const double min_dist = 0.0;
  const double max_dist = 5.0;
  int size = distance_buffer_all_.size();

  for(int i = 1; i < size; i++){
    Eigen::Vector2d coord = gridIndex2coordd(vectornum2gridIndex(i));
    pcl::PointXYZI pt;
    pt.x = coord.x(); pt.y = coord.y(); pt.z = 0.0;
    pt.intensity = std::max(min_dist, std::min(distance_buffer_all_[i], max_dist));
    cloud_vis.points.push_back(pt);
  }

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "map";
  pub_ESDF_.publish(surf_vis);
}

void SDFmap::publish_Neighboring_PC(){
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  const double r1 = r_min_ * r_min_;
  const double r2 = r_max_ * r_max_;

  const int size = distance_buffer_all_.size();
  if(size <= 1) return;

  cloud_vis.points.reserve(size / 5);

  for(int i = 1; i < size; i++){
    const double d = distance_buffer_all_[i];
    if(d <= safe_dis_) continue;

    const Eigen::Vector2d coord = gridIndex2coordd(vectornum2gridIndex(i));

    const double dx = coord.x() - odom_pos_.x();
    const double dy = coord.y() - odom_pos_.y();
    if(dx * dx + dy * dy > r2 || dx * dx + dy * dy < r1) continue;

    pcl::PointXYZ pt;
    pt.x = coord.x();
    pt.y = coord.y();
    pt.z = 0.0;
    cloud_vis.points.push_back(pt);
  }

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;

  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "map";
  surf_vis.header.stamp = ros::Time::now();
  pub_NeiPC_.publish(surf_vis);
}

inline double SDFmap::getDistance(const Eigen::Vector2i& id){
  return distance_buffer_all_[Index2Vectornum(id[0], id[1])];
}

inline double SDFmap::getDistance(const int& idx, const int& idy){
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
}

inline Eigen::Vector2i SDFmap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((pt(0) - global_x_lower_) * inv_grid_interval_ - 0.5), 0), GLX_SIZE_ - 1),
         std::min(std::max(int((pt(1) - global_y_lower_) * inv_grid_interval_ - 0.5), 0), GLY_SIZE_ - 1);
  return idx;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x() < global_x_lower_ || pos.y() < global_y_lower_ || pos.x() > global_x_upper_ || pos.y() > global_y_upper_){
    grad.setZero();
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x() >= GLX_SIZE_ - 1 || idx.y() >= GLY_SIZE_ - 1){
    grad.setZero();
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double& mindis){
  if(pos.x() < global_x_lower_ || pos.y() < global_y_lower_ || pos.x() > global_x_upper_ || pos.y() > global_y_upper_){
    grad.setZero();
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x() >= GLX_SIZE_ - 1 || idx.y() >= GLY_SIZE_ - 1){
    grad.setZero();
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  if(dist > mindis){
    return dist;
  }

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

double SDFmap::getDistWithGradBilinear(const Eigen::Vector2d &pos){
  if(pos.x() < global_x_lower_ || pos.y() < global_y_lower_ || pos.x() > global_x_upper_ || pos.y() > global_y_upper_){
    return 1e10;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos_m);
  if(idx.x() >= GLX_SIZE_ - 1 || idx.y() >= GLY_SIZE_ - 1){
    return 1e10;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = getDistance(current_idx);
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  return dist;
}

double SDFmap::getDistanceReal(const Eigen::Vector2d& pos){
  if(pos.x() < global_x_lower_ || pos.y() < global_y_lower_ || pos.x() > global_x_upper_ || pos.y() > global_y_upper_){
    return 10000;
  }
  Eigen::Vector2i idx = coord2gridIndex(pos);
  return distance_buffer_all_[idx.x() * GLY_SIZE_ + idx.y()];
}

double SDFmap::getUnkonwnGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad){
  if(pos.x() < global_x_lower_ || pos.y() < global_y_lower_ || pos.x() > global_x_upper_ || pos.y() > global_y_upper_){
    grad.setZero();
    return 100;
  }
  Eigen::Vector2d pos_m = pos;
  Eigen::Vector2i idx = coord2gridIndex(pos_m);
  if(idx.x() >= GLX_SIZE_ - 1 || idx.y() >= GLY_SIZE_ - 1){
    grad.setZero();
    return 100;
  }

  Eigen::Vector2d idx_pos = gridIndex2coordd(idx);
  Eigen::Vector2d diff = (pos - idx_pos) * inv_grid_interval_;

  double values[2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
      values[x][y] = gridmap_[Index2Vectornum(current_idx)] == Unknown ? 1 : 0;
    }
  }

  double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
  double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
  double dist = (1 - diff[1]) * v0 + diff[1] * v1;

  grad[1] = (v1 - v0) * inv_grid_interval_;
  grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) + diff[1] * (values[1][1] - values[0][1])) * inv_grid_interval_;

  return dist;
}

Eigen::Vector2d SDFmap::get_update_odom(){
  return update_odom_;
}

inline double SDFmap::normalize_angle(double angle){
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool SDFmap::isOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Occupied;
}

bool SDFmap::isOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
}

bool SDFmap::isUnOccupied(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
}

bool SDFmap::isUnOccupied(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
}

bool SDFmap::isUnknown(const Eigen::Vector2i &index){
  return gridmap_[Index2Vectornum(index)] == Unknown;
}

bool SDFmap::isUnknown(const int &idx, const int &idy){
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
}

bool SDFmap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
}

bool SDFmap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(idx, idy)] < safe_dis;
}

void SDFmap::publish_ESDFGrad(){
  visualization_msgs::MarkerArray grad_all;

  Eigen::Vector2i min_cut(0,0);
  Eigen::Vector2i max_cut(GLX_SIZE_ - 1, GLY_SIZE_ - 1);

  for (int x = min_cut(0) + 2; x < max_cut(0); x += 5)
    for (int y = min_cut(1) + 2; y < max_cut(1); y += 5) {
      Eigen::Vector2d pos = gridIndex2coordd(x, y);
      Eigen::Vector2d d(0.025, 0.025);
      pos = pos + d;
      Eigen::Vector2d grad;
      getDistWithGradBilinear(pos, grad);

      visualization_msgs::Marker grad_Marker;
      grad_Marker.header.frame_id = "map";
      grad_Marker.header.stamp = ros::Time::now();
      grad_Marker.ns = "map";
      grad_Marker.type = visualization_msgs::Marker::ARROW;
      grad_Marker.action = visualization_msgs::Marker::ADD;
      grad_Marker.id = (x + 1) + (y * 100000);
      grad_Marker.pose.position.x = pos[0];
      grad_Marker.pose.position.y = pos[1];
      grad_Marker.pose.position.z = 0.0;

      Eigen::Quaterniond Quat;
      Eigen::Vector3d vectorBefore(1, 0, 0);
      Eigen::Vector3d vectorAfter(grad.x(), grad.y(), 0.0);
      Quat = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);

      grad_Marker.pose.orientation.w = Quat.w();
      grad_Marker.pose.orientation.x = Quat.x();
      grad_Marker.pose.orientation.y = Quat.y();
      grad_Marker.pose.orientation.z = Quat.z();

      grad_Marker.scale.x = (grad.norm() + 0.01) / 10.0;
      if(grad.norm() > 1000){
        ROS_INFO("grad error!!!  position: %f  %f    grad:%f  %f   grad.norm(): %f",
                 pos.x(), pos.y(), grad.x(), grad.y(), grad.norm());
      }
      grad_Marker.scale.y = 0.01;
      grad_Marker.scale.z = 0.01;

      grad_Marker.color.r = 0.0f;
      grad_Marker.color.g = 1.0f;
      grad_Marker.color.b = 0.0f;
      grad_Marker.color.a = 1.0;
      grad_Marker.lifetime = ros::Duration(100.0);
      grad_Marker.frame_locked = true;

      grad_all.markers.push_back(grad_Marker);
    }

  pub_gradESDF_.publish(grad_all);
}