# multi_slam_karto

Multi-robot SLAM tailored for Realm project.

> This repo is modified based on [multi_slam_karto](https://github.com/SunZezhou/multi_slam_karto).



## Overview

This is a centralized multi-robot SLAM implementation. All scans of robots are optimized in a graph, instead of using the map-merge method to generate global map.

This work contains a modified version of [SLAM Karto](https://github.com/ros-perception/slam_karto) and [Open Karto](https://github.com/ros-perception/open_karto). 

The gazebo environment uses [rrt_exploration_tutorials](https://github.com/hasauino/rrt_exploration_tutorials).

![multi_slam_karto](./demo/demo.gif)

## Prerequisites

This package has been tested on Ubuntu16.04 with ROS Kinetic. As far as I know, Kobuki packages in step 4 can't be installed correctly on ROS Melodic. 

(1) You should have installed a ROS distribution with gazebo.

(2) Install ROS navigation stack.

`$ sudo apt-get install ros-kinetic-navigation`

(3) Install eigen.

(4) Install sparse-bundle-adjustment

`$ sudo apt-get install ros-kinetic-sparse-bundle-adjustment`

(5) Install Kobuki robot packages

`sudo apt-get install ros-kinetic-kobuki ros-kinetic-kobuki-core ros-kinetic-kobuki-gazebo`

## Usage

(1) Compile

`$ catkin_make`

(2) Run

`$ roslaunch multi_slam_karto multi_robots.launch`

(3) Publish movebase goal
```
rostopic pub /robot_1/move_base_simple/goal geometry_msgs/PoseStamped "
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose: 
    position: 
      x: 3.0
      y: 3.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

rostopic pub /robot_2/move_base_simple/goal geometry_msgs/PoseStamped "
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose: 
    position: 
      x: 3.0
      y: -3.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

rostopic pub /robot_3/move_base_simple/goal geometry_msgs/PoseStamped "
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose: 
    position: 
      x: -3.0
      y: 3.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"

rostopic pub /robot_4/move_base_simple/goal geometry_msgs/PoseStamped "
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose: 
    position: 
      x: -3.0
      y: -3.0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
"
```

## Update

- 2020.08.19  Add real robot launchfile.  Note: I modified the hokuyo driver. You can set the topic of the lidar scan in the launchfile. Git here [urg_node](https://github.com/SunZezhou/urg_node_my_config).

