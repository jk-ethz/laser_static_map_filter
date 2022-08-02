# Laser Static Map Mask/Subtraction Filter

Subtract a static map (`OccupancyGridMap`) from a laser scan or point cloud. This can be used for example to remove obstacles from a point cloud or 2D lidar scan. The map serves as a masking layer for the point cloud.

This package is intended to be used for example in conjunction with [obstacle_detector](https://github.com/jk-ethz/obstacle_detector).

If you only got a `sensor_msgs::LaserScan`, you can use the `scans_merger` node in the `obstacle_detector` package to get a `sensor_msgs::PointCloud` msg, which is required as an input to this package.

![Removing obstacles in a static map (black) from a laser scan point cloud](assets/example.png)

## Installation

```bash
cd catkin_ws/src
git clone https://github.com/jk-ethz/laser_static_map_filter
vcs import --recursive --input laser_static_map_filter.repos
catkin build
```

## Usage

```bash
rosrun laser_static_map_filter laser_static_map_filter.py
```

An example launch file is provided as well

```bash
roslaunch laser_static_map_filter laser_static_map_filter.launch
```
