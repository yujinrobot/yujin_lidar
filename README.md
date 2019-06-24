![lidar](https://upload.wikimedia.org/wikipedia/commons/2/22/Yujin_lidar.jpg "Yujin Lidar")
# Yujin LiDAR

## About Yujin LiDAR

Yujin 3D time-of-flight LiDAR is designed to measure distance from surroundings and collect 3D point cloud data. LiDAR is an optimized solution of indoor mapping, navigation, localization and other applications in the field of robotics, industries and safety & security.
## Yujin LiDAR Software Maintenance

- ROS Version: Melodic
- Maintainer Status: developed
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
### yujin_lidar
yujin_lidar
#### published Topics
- yujin_3d_pub
#### Parameters
- ip (string, default:"")
- lk_file
-

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
