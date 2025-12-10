# Second Robotics Project

## Overview
This ROS package implements autonomous navigation for a skid-steering robot using 2D and 3D laser scanner data for mapping and navigation.

## Package Structure
```
second_project/
├── cfg/
├── launch/
│   ├── mapping.launch
│   └── navigation.launch
├── src/
│   ├── tf_publisher.cpp
│   ├── pointcloud_to_laserscan.cpp
│   └── navigation.cpp
├── srv/
├── map_raw/
│   ├── map_2d_single.yaml
│   ├── map_2d_single.pgm
│   ├── map_2d_velodyne.yaml
│   └── map_2d_velodyne.pgm
├── stage/
│   └── world.world
├── waypoints.csv
├── config_mapping.rviz
├── config_nav.rviz
├── CMakeLists.txt
└── package.xml
```

## Dependencies
- ROS Melodic/Noetic
- gmapping or slam_gmapping
- move_base
- pointcloud_to_laserscan
- stage_ros
- tf
- sensor_msgs
- geometry_msgs
- nav_msgs

## Mapping Process

### Launch Mapping
```bash
roslaunch second_project mapping.launch
```

In another terminal, play the bag file:
```bash
rosbag play --clock first.bag
```

This will:
- Publish TF transformations between odom and laser frames
- Convert 3D Velodyne data to 2D laser scan
- Run gmapping to create maps from both single-plane and multi-plane laser data
- Display the mapping process in RViz

### Generated Maps
Four maps are generated:
1. Map from single-plane laser (/scan)
2. Map from single-plane laser (alternative configuration)
3. Map from Velodyne 3D laser converted to 2D
4. Map from Velodyne (alternative configuration)

## Navigation

### Stage Simulation Setup
The stage simulation uses the generated map and simulates the robot environment with realistic sensor characteristics.

### Launch Navigation
```bash
roslaunch second_project navigation.launch
```

This will:
- Start the stage simulator with the generated map
- Launch move_base with appropriate local and global planners
- Start the navigation node that reads waypoints from CSV
- Display navigation in RViz

### Waypoints Format
The `waypoints.csv` file contains waypoints in the format:
```
x1,y1,theta1
x2,y2,theta2
x3,y3,theta3
```

Where:
- x, y are coordinates in the map frame (meters)
- theta is the heading angle (radians)

## Nodes Description

### tf_publisher
Converts odometry data from `/t265/odom` to TF transformations. Publishes the transform between `odom` and `base_link` frames.

### pointcloud_to_laserscan (optional)
If using a custom node, it converts `/velodyne_points` (PointCloud2) to a LaserScan message for 2D mapping. Alternatively, the pointcloud_to_laserscan package can be used.

### navigation
Reads waypoints from the CSV file and sequentially publishes goals to move_base. When a goal is reached, it publishes the next waypoint until all are completed.

## Configuration

### Mapping Configuration
- SLAM algorithm: gmapping
- Map resolution: 0.05m
- Update interval: 1.0s
- Laser range: depends on sensor

### Navigation Configuration
- Global planner: navfn or global_planner
- Local planner: DWA (dwa_local_planner) or TEB
- Costmap parameters tuned for 0.6m x 0.4m footprint

## Usage Instructions

1. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. For mapping:
```bash
roslaunch second_project mapping.launch
rosbag play --clock first.bag
```

3. Save maps when satisfied:
```bash
rosrun map_server map_saver -f map_single
rosrun map_server map_saver -f map_velodyne
```

4. Edit waypoints.csv with desired navigation goals

5. For navigation:
```bash
roslaunch second_project navigation.launch
```

## Notes
- TF between odometry and lasers assumed at origin (0,0,0)
- Minimal post-processing allowed on maps (noise removal)
- Original unprocessed maps provided in map_raw folder
- Robot footprint: 0.6m x 0.4m (skid steering)