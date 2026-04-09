cmake_minimum_required(VERSION 3.0.2)
project(traj_map_visualizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  rviz
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/traj_visualizer.py
  scripts/record_speed_plot.py
  record_los_min_plot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)