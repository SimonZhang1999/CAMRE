cmake_minimum_required(VERSION 3.0.2)

project(cure_planner)

#for Robot class
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CAMKE_CXX_FLAGS_RELEASE "-03 -wall -g -pthread")
#add_compile_options(-std=c++11)
add_library(frontier_assigner_lib
  src/exploration/FrontierAssigner.cpp
  src/exploration/star_convex_handle.cpp
)
add_library(graph_mst_lib
  src/exploration/GraphMST.cpp
  src/exploration/star_convex_handle.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  cv_bridge 
  image_transport 
  roscpp 
  sensor_msgs 
  laser_geometry
  std_msgs 
  tf 
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# OpenCV is required for merging without initial positions
find_package(OpenCV 4 REQUIRED)
 if(OpenCV_VERSION VERSION_LESS "4.0")
  message(FATAL_ERROR "This package needs OpenCV >= 4.0")
 endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} include) 


add_message_files(FILES
  PointArray.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime)



add_executable(frontiers
  src/exploration/frontiers.cpp
)
add_dependencies(frontiers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontiers
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(get_voronoi_centroid
  src/exploration/get_voronoi_centroid.cpp
)
add_dependencies(get_voronoi_centroid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_voronoi_centroid
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(robot_position
  src/exploration/robot_position.cpp
)
add_dependencies(robot_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_position
  ${catkin_LIBRARIES}
)

add_executable(target_assigner
  src/exploration/target_assigner.cpp
)
add_dependencies(target_assigner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_assigner
  ${catkin_LIBRARIES}
)

add_executable(merge_map
 src/exploration/merge_map.cpp
)
add_dependencies(merge_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_map
 ${catkin_LIBRARIES}
)

add_executable(graph_mst
 src/exploration/GraphMST.cpp
)
add_dependencies(graph_mst ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(graph_mst graph_mst_lib
 ${catkin_LIBRARIES}
)

add_executable(frontier_assigner_node
 src/exploration/FrontierAssigner.cpp 
)
add_dependencies(frontier_assigner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_assigner_node frontier_assigner_lib
 ${catkin_LIBRARIES}
)

add_executable(scan_filter src/scp/scan_filter.cpp)
target_link_libraries(scan_filter ${catkin_LIBRARIES})

add_executable(pub_flipped_convexhull src/scp/pub_flipped_convexhull.cpp)
target_link_libraries(pub_flipped_convexhull ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(star_convex_handle src/scp/star_convex_handle.cpp)
# target_link_libraries(star_convex_handle ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(scan_star_region_node src/scp/scan_star_region_node.cpp)
target_link_libraries(scan_star_region_node ${catkin_LIBRARIES})