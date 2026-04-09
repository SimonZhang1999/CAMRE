cmake_minimum_required(VERSION 2.8.3)
project(multi_slam_realm)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
#    open_karto
    laser_geometry
    rosconsole
    roscpp
    rospy
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
    std_msgs
    message_generation
    geometry_msgs
    dynamic_reconfigure
)
# include_directories("/usr/local/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)   # 将multi_slam_karto/include下的头文件，添加到src的头文件搜索目录中


find_package(Eigen REQUIRED)


add_message_files(
  FILES
  PointArray.msg
)

# Dependence to generate self-defined messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/informationGain.cfg
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)


include_directories(${catkin_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


install(PROGRAMS 
    scripts/opencv_detector.py
    scripts/filter.py 
    scripts/initializer.py 
    scripts/util_modules/constants.py 
    scripts/util_modules/functions.py  
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(multi_slam_karto src/slam_karto.cpp src/spa_solver.cpp src/Karto.cpp src/Mapper.cpp)
target_link_libraries(multi_slam_karto ${catkin_LIBRARIES})

# install(TARGETS multi_slam_karto
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


add_executable(d_opt_global_rrt_detector src/frontier_detector/global_rrt_detector.cpp src/frontier_detector/functions.cpp src/frontier_detector/mtrand.cpp)
target_link_libraries(d_opt_global_rrt_detector ${catkin_LIBRARIES})

add_executable(d_opt_local_rrt_detector src/frontier_detector/local_rrt_detector.cpp src/frontier_detector/functions.cpp src/frontier_detector/mtrand.cpp)
target_link_libraries(d_opt_local_rrt_detector ${catkin_LIBRARIES})

add_executable(scan_filter src/filterScan.cpp)
target_link_libraries(scan_filter ${catkin_LIBRARIES})




