# gridmap_to_pointcloud_ros

![Build Status](https://github.com/ToshikiNakamura0412/gridmap_to_pointcloud_ros/workflows/build/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS package for converting gridmap to pointcloud

<p align="center">
  <img src="images/gridmap_to_pointcloud.png" height="320px"/>
</p>

## Environment
- Ubuntu 20.04
- ROS Noetic

## Install and Build
```
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/gridmap_to_pointcloud_ros.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic           # Install dependencies
catkin build gridmap_to_pointcloud_ros -DCMAKE_BUILD_TYPE=Release # Release build is recommended
```

## Running the demo
```
roslaunch gridmap_to_pointcloud_ros test.launch
```

## Node I/O
![Node I/O](images/node_io.png)

## Nodes
### gridmap_to_pointcloud
#### Published Topics
- /map_cloud (`sensor_msgs/PointCloud2`)
  - Pointcloud converted from gridmap

#### Parameters
- ~\<name>/<b>hz</b> (int, default: `1` [Hz]):<br>
  The rate of main loop
