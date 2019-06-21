# yujin_lidar
all sources for the Yujin lidar
# Yujin LiDAR

##About Yujin LiDAR

##Yujin LiDAR Software Maintenance

- ROS Version: Melodic
- Maintainer Status: developed
- Author: Ju Young Kim
- License: BSD

## Supported Hardware

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
####published Topics
- yujin_3d_pub
####Parameters
- ip (string, default:"")
- 
-

## QUICK START
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
