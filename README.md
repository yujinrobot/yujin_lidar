![lidar](https://upload.wikimedia.org/wikipedia/commons/2/22/Yujin_lidar.jpg "Yujin Lidar")
# Yujin LiDAR

## About Yujin LiDAR

Yujin time-of-flight LiDAR is designed to measure distances from surroundings and collect point cloud data. Yujin LiDAR is an optimized solution of indoor mapping, navigation, localization and other applications in the field of robotics, industries and safety & security.

## ROS Package Maintenance

- ROS Version: Melodic
- Maintainer Status: Developed
- Author: Ju Young Kim
- License: BSD

## Supported Hardware

- YRL2-05 (2D, 5m)
- YRL2-25 (2D, 25m)
- YRL3-05 (3D, 5m)
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
- Package Name: yujin_lidar
- Node Name: yujin_pub
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
rosrun yujin_lidar yujin_pub
rostopic echo /yujin3d/yujin_cloud
rosrun rviz rviz
```
## Additional Software
- Viewer
- IP Address Chaning Tool
## Documents
- Catalog
- User manual
- Communication Protocol Specification
- 2D/3D CAD

------------------------------------------------------------------------
![Yujin Logo](https://upload.wikimedia.org/wikipedia/commons/0/0f/Yujinrobot_logo.png "Yujin Logo")
