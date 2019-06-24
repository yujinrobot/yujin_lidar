![lidar](https://upload.wikimedia.org/wikipedia/commons/2/22/Yujin_lidar.jpg "Yujin Lidar")
# Yujin LiDAR

## About Yujin LiDAR

Yujin time-of-flight LiDAR is designed to measure distances from surroundings and collect point cloud data. Yujin LiDAR is an optimized solution of indoor mapping, navigation, localization and other applications in the field of robotics, industries and safety & security.

## Yujin LiDAR Software Maintenance

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
### Parameters
#### ip_address (string, default:"192.168.1.251")
- "ip_address" is an ip address of a LiDAR
#### resolution (double, default: 0.1756097561)
- "resolution" is an angle offset that is added to vertical scanning after covering one vertical range.
#### noise_level (float, default: 0.95)
- Raw data from a LiDAR contains some non-object data (which is an error). "noise level" is a filter that removes those sparse errors.
#### min_height (double, default: 0)
- "min_height" is the minimum level that is displayed in the screen.
#### max_height (double, default: 3)
- "max_height is the maximum level that is displayed in the screen.
#### cutoff_height (double, default: 3)
- "cutoff_height is the upper level where the data is cut in the screen. 
#### sensor_height (double, default: 1.2)
- "sensor_height" is the height level where a LiDAR is placed.
#### max_vertical_angle (double, default: 0.7854)

- min_vertical_angle (double, default: -0.7854)
- max_horizontal_angle (double, default: 1.570796)
- min_horizontal_angle (double, default: -1.570796)
- angle_offset (float, default: 2.530727)

## QUICK START
filepath_to_lk.bin should be the file path to lk.bin.
```bash
rosrun yujin_3d_lidar yujin_3d_pub _lk_file:=filepath_to_lk.bin
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
