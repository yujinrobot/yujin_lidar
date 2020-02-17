# Yujin LiDAR
Official Website: http://lidar.yujinrobot.com/

## About Yujin LiDAR

YRL series LiDAR is designed to detect objects, measure distances from surroundings and collect data as point clouds. Yujin LiDAR is an optimized solution for indoor mapping, navigation, localization and other applications in a variety of industries including robotics, safety and security.
## Videos of 3D SLAM and Obstacle Detection
![](slam_F1.gif)
![](slam_F1_2.gif)
![](od_1.gif)
![](od_2.gif)
![](od_3.gif)

## ROS Package Maintenance

- ROS Version: Melodic
- Maintainer Status: Developed
- Author: Ju Young Kim
- License: BSD

![](ros_driver.gif)

## Supported Hardware

- YRL2-05 (2D, 5m)
- YRL2-15 (2D, 15m)
- YRL2-25 (2D, 25m)
- YRL3-05 (3D, 5m)
- YRL3-15 (3D, 15m)
- YRL3-25 (3D, 25m)

## ROS Package Installation

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone https://
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
rospack profile
```

## ROS API
- Package Name: yujin_yrl3_package
- Node Name: yrl3_pub
- Publisher Name : yrl3_pub
- Topic Name : yrl3_cloud
### Parameters (Type, Default value, Unit)
If you do not specify parameters' values, default values will be used. Parameters with name "height" require exact values to be put in order to produce an exact picture of surroundings.
#### ip_address (string, "192.168.1.251", N/A)
- An IP address of your LiDAR
#### resolution (double, 0.1756097561, degree)
- An angle offset that is added to vertical scanning after covering one vertical range.
#### noise_level (float, 0.95, N/A)
- Raw data from a LiDAR contains some non-object data (which is an error). This is a filter that removes those sparse errors.
#### min_height (double, 0, meter)
- The minimum height level that is displayed on the screen.
#### max_height (double, 3, meter)
- The maximum height level that is displayed on the screen.
#### cutoff_height (double, 3, meter)
- The upper height level where the data is cut on the screen. 
#### sensor_height (double, 1.2, meter)
- The height level where a LiDAR is placed.
#### max_vertical_angle (double, 0.7854, radian)
- The upper angle range of laser scan. The default value is +90 in degree.
#### min_vertical_angle (double, -0.7854, radian)
- The lower angle range of laser scan. The default value is -90 in degree.
#### max_horizontal_angle (double, 1.570796, radian)
- The right angle range of laser scan. The default value is +180 in degree.
#### min_horizontal_angle (double, -1.570796, radian)
- The left angle range of laser scan. The default value is -180 in degree.
#### angle_offset (float, 0, radian)
- A horizontal angle offset that is added to the point cloud data. Zero is recommended.

## QUICK START
```bash
rosrun yujin_yrl3_package yrl3_pub
rostopic echo /yrl3_pub/yrl3_cloud
rosrun rviz rviz
```
## Additional Software
### Viewer
- Ubuntu 18.04 is required.
#### For Linux: Dependency Installation
```bash
sudo apt-get install qt5-default
```
#### For Linux: Quick Start For Viewer
```bash
cd dir_of_Yujin_Lidar_Viewer
sudo chmod 777 Yujin_Lidar_Viewer
./Yujin_Lidar_Viewer
```

- IP Address Chaning Tool
## Documents
- Catalog
- User manual
- Communication Protocol Specification
- 2D/3D CAD

------------------------------------------------------------------------
![lidar](https://upload.wikimedia.org/wikipedia/commons/2/22/Yujin_lidar.jpg "Yujin Lidar")
![Yujin Logo](https://upload.wikimedia.org/wikipedia/commons/0/0f/Yujinrobot_logo.png "Yujin Logo")
