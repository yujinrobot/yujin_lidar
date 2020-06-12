YUJIN ROBOT Co.,Ltd. Ju Young Kim (jykim3 [at] yujinrobot.com)

* This YRL_SW_Package contains total 5 SWs.
1) Windows 10 Driver
2) Windows 10 Viewer
3) Ubuntu 18.04 Driver
4) Ubuntu 18.04 Viewer
5) Ubuntu 18.04 ROS Melodic Driver

* SW Common Notice
1) These SWs support only 64 bit environment.
2) Connect the product to a power source, and to a computer using ethernet cable. Set your computer ethernet network address (IPv4) to 192.168.1.x and subnet mask to 255.255.255.0. (YRL Series LiDAR has a default ip address, 192.168.1.250)
3) Calibration file is a unique data file for each LiDAR product to measure distances, so it is essential to import a calibration file at first. Each LiDAR product has its own unique calibration file. The name of the calibration file will be lk+serial number(12 digits).bin
4) These SWs all need to refer a calibration file of a certain LiDAR sample. Each LiDAR sample's calibration file can be created by "Create a Calibration File" button of Windows 10/Ubuntu 18.04 Viewer.
- Viewers can refer to a calibration file by "Import a Calibration File" button.
- Drivers can refer to a calibration file's address by using setCalibrationFilePath(std::string cal_file_path) function.

* Viewer Notice
0) For Ubuntu 18.04 Viewer, qt5 package should be installed.
<qt5 package Installation>
	1> sudo apt-get install qt5-default
1) Run as an administrator
e.g) In Ubuntu 18.04 
sudo -H ./Yujin_Lidar_Viewer.sh
2) Through "Create a Calibration File" and "Import a Calibration File", LiDAR's data can be visualized.
3) Set LiDAR Position Z as an actual height of a LiDAR where lazer is being emitted in order to get a realistic visualization of data. (When the product is on a floor, Z is 0.06m)

* Driver Notice
In case of 2D, filter is automatically fixed to 0.01, sensor height to 0.2 and functions related to vertical level are automatically disabled. Also when using 2D LiDAR, please ignore vertical angle values of getSphericalOutputs functions and z values of getCartesianOutputs function.

* Windows 10 Driver Build Environment
Windows 10 Driver for YRL Series is written in C++, and was compiled using : 

Windows SDK Version : 10.0.16299.0
Visual Studio 2015 C++ compiler (from MSVC v140 - VS2015 C++ Build Tools v14.00 for x64)

Driver compiled by this compiler, has a compatibility with applications that are built by v140, v141, v142 Visual Studio C++ compiler.
Driver compiled with this version of Windows SDK, has a compatibility with applications that are built with Windows SDK released after the releasement of this version of Windows SDK.
Final mutual compatibility of a compiler and SDK being used, needs to be considered by a user.

* Ubuntu 18.04 Driver Build Environment
Ubuntu 18.04 Driver for YRL Series is written in C++, and was compiled using : 

g++ (Ubuntu 7.4.0-1ubuntu1~18.04.1) 7.4.0 (x64)

Linking pthread is essential for successful build.

* Driver Examples Codes Explanation
1) test_yrl_library.cpp
- This can read and write basic parameters of the driver.
- getSphericalOutputs can read output data that are not processed by basic parameters except a filter.
- getCartesianOutputs can read output data that are processed by basic parameters such as sensor height, upper/lower data limit, max/min vertical/horizontal angle, and etc.

e.g) getSphericalOutputs : Horizontal angle 0.3 rad, Vertical angle 1.4 rad, 3 meter.
e.g) getCartesianOutputs : (3m, 2m, 5m)

2) test_fw_interface.cpp
- This can read and write basic firmware parameters that are supported by the driver.
fwGetYrlIpAddress and fwGetYrlVerticalAngles read LiDAR's current IP address and vertical angle range.
fwSetYrlIpAddress sets LiDAR's IP address to other address.

- fwSetYrlVerticalAngles sets LiDAR's vertical angle ranges.
When this function is used, Lidar turns on and off and it takes 13 seconds for the LiDAR to get back to normal operation.
Later version of driver will be able to change LiDAR's vertical angle ranges in much shorter time.

3) test_distance_reader.cpp
- This can test basic ranging of YRL series LiDARs.
This prints out range values of detected objects within plus minus 5 degrees of horizontal range and vertical range.

4) test_error_code.cpp
- This can test basic error code of YRL series LiDARs.
When started, error H is automatically created. This peridically gets error values using getErrorCode function, and when there are errors, it prints logs related to errors.

* ROS Melodic Driver Notice
1) Please refer to the guide in : https://github.com/yujinrobot/yujin_lidar


