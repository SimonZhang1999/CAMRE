<div align ="center">
<h3> CASE 2026: Multi-Nonholonomic Robot Exploration with Line-of-Sight Maintenance in Unknown Environments </h3>

Weijian Zhang, Charlie Street, Masoumeh Mansouri

University of Birmingham

<a href="https://research.birmingham.ac.uk/en/publications/multi-nonholonomic-robot-exploration-with-line-of-sight-maintenan/"><img alt="Paper" src="https://img.shields.io/badge/Paper-Preprint-pink"/></a>
<a href="https://youtu.be/gtUvCmZlF74"><img alt="Video" src="https://img.shields.io/badge/Video-Youtube-red"/></a>
</div>

## Overview
<p align="center">
  <img src="https://github.com/HyPAIR/CAMRE/blob/main/misc/case2026.gif" alt="CAMRE" width="650">
</p>

## Features

<p align="center">
  <img src="https://github.com/HyPAIR/CAMRE/blob/main/misc/proposed%20framework.png" alt="exploration_framework" height=350">
</p>

 - We develop a cooperative exploration framework that enables teams of nonholonomic robots to explore unknown environments while maintaining LoS connectivity.
 - We incorporate LoS visibility constraints derived from onboard LiDAR observations directly into distributed trajectory optimization, allowing robots to generate dynamically feasible motions that maintain LoS connectivity while respecting collision and communication constraints.
 - We validate our approach in both simulation and hardware.

## Requirements

 - ROS Noetic or later
 - Ubuntu 20.04 or later

## Installation

```bash
# Create a new workspace:
mkdir -p ~/CAMRE/src
cd ~/CAMRE/src
catkin_init_workspace

# Clone the package into the workspace
git clone git@github.com:HyPAIR/CAMRE.git

## Quick Start
```bash
# Build
cd ~/CAMRE
catkin_make

# Terminal 1 - Launch simulation & exploration framework
source devel/setup.bash
cd src/CAMRE
./env.sh

# Terminal 2 - Launch trajectory planner
source devel/setup.bash
cd src/CAMRE
./planner.sh
```

## Video

A simulation and real-world experiments video demonstrating our proposed framework can be found at [bilibili](https://www.bilibili.com/video/BV1W2wUzAEod/?spm_id_from=333.1387.list.card_archive.click).

### Acknowledgement
We build upon [**DDR-opt**](https://github.com/ZJU-FAST-Lab/DDR-opt) to achieve efficient distributed trajectory optimization. We use [**active_graph_slam**](https://github.com/JulioPlaced/active_graph_slam) for frontier detection, and [**LoS_constrained_navigation**](https://github.com/bairuofei/LoS_constrained_navigation) to generate visible regions.
