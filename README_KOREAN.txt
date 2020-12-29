(주)유진로봇 김주영 개발자 (jykim3 [at] yujinrobot.com)

S/W 사용시 문의 사항은 해당 메일로 부탁드립니다.
한국 : andyjun [at] yujinrobot.com
해외 : hjyi [at] yujinrobot.com

* 해당 YRL_SW_Package 는 
1) Windows 10 Driver
2) Windows 10 Viewer
3) Ubuntu 18.04 Driver
4) Ubuntu 18.04 Viewer
5) Ubuntu 18.04 ROS Melodic Driver
총 5개의 SW를 포함하고 있다.

* SW 공통 유의사항
1) 모든 SW는 64비트 환경만을 지원한다.
2) YRL 라이다에 파워를 주고 이더넷 포트에 케이블을 연결한 뒤, 컴퓨터 이더넷 네트워크를 수동으로 192.168.1 점대로 맞춘다. (YRL 시리즈의 기본 IP 주소는 192.168.1.250 이다.)
3) 켈리브레이션 파일은 각 라이다 시료의 거리를 재기위한 고유의 데이터 파일이다. 모든 라이다는 자기 고유의 켈리브레이션 파일을 가지고 있으며, 케리브레이션 파일의 이름은 lk+시리얼번호(12자리).bin 이다.
4) 해당 SW들은 다 해당 시료의 calibration file 의 참조를 필요로 한다. 해당 시료의 calibration file 은 Windows 10/Ubuntu 18.04 Viewer의 켈리브레이션 파일 생성버튼을 통해 생성할 수 있다.
- Viewer는 켈리브레이션 파일 참조버튼을 통해 참조할 수 있다.
- Driver는 setCalibrationFilePath(std::string cal_file_path) 함수를 통해 켈리브레이션 파일의 주소를 참조할 수 있다. 

* Viewer 유의사항
0) Ubuntu 18.04 Viewer 는 qt5 pacakge를 깔아야 한다.
<qt5 package Installation>
	1> sudo apt-get install qt5-default
1) 뷰어 실행 시 관리자 권한으로 실행하는 것을 권장한다.
e.g) Ubuntu 18.04 에서 
sudo -H ./Yujin_Lidar_Viewer.sh
2) 켈리브레이션 파일 생성과 참조를 통해 해당 라이다의 데이터를 볼 수 있다.
3) 라이다 위치의 Z 값을 실제 레이저가 나오는 센서의 높이로 주어야 실제와 합하는 visualized data를 볼 수 있다. (라이다가 땅에 있을경우 0.06m 이다.)

* Driver 유의사항
2D 의 경우 Filter 가 0.01로, 센서위치가 0.2로 고정되고 수직레벨에 관련된 함수들이 자동으로 무시된다. 또한 2D의 경우 getSphericalOutputs 계열 데이터 출력 함수들의 수직각도 값과 getCartesianOutputs 데이터 출력 함수의 Z값을 무시하면 된다.

* Windows 10 드라이버 빌드 환경
YRL 시리즈를 위한 본 Windows 10 Driver 는 C++ 로 쓰여졌으며,
Windows SDK Version : 10.0.16299.0
Visual Studio 2015 C++ compiler (from MSVC v140 - VS2015 C++ Build Tools v14.00 for x64)
를 사용하여 빌드되었다.
해당 compiler로 빌드된 드라이버는 독립적으로 v140, v141, v142 Visual Studio C++ compiler로 빌드되는 application과 호환성을 가진다.
해당 Windows SDK 로 빌드된 드라이버는 독립적으로 해당 SDK 이후 출시된 Windows SDK로 빌드되는 application과 호환성을 가진다.
최종으로 사용되는 compiler와 SDK의 상호호환성은 사용자가 고려해야 한다.

* Ubuntu 18.04 드라이버 빌드 환경
YRL 시리즈를 위한 본 Ubuntu 18.04 Driver 는 C++ 로 쓰여졌으며,
g++ (Ubuntu 7.4.0-1ubuntu1~18.04.1) 7.4.0 (x64) 를 사용하여 빌드되었다.
pthread 를 링크하여야 빌드가 가능하다.

* 드라이버 예제 설명
1) test_yrl_library.cpp
- 기본적인 드라이버의 parameter 를 읽어오고 쓸 수 있다. 
getSphericalOutputs 계열 데이터 출력 함수들은 가공되지 않은 data를 출력한다.
getCartesianOutputs 데이터 출력 함수는 sensor height, upper/lower data limit, max/min vertical/horizontal angle 등의 parameter로 가공 후 data를 출력한다.
ex) getSphericalOutputs : 수평각도 0.3 radian 수직각도 1.4 radian, 3미터 거리에 있는 포인트 클라우드
getCartesianOutputs : (3m, 2m, 5m) 지점의 포인트 클라우드

2) test_fw_interface.cpp
- 기본적인 YRL 시리즈의 펌웨어의 parameter를 읽어오고 쓸 수 있다.
fwGetYrlIpAddress, fwGetYrlVerticalAngles 는 라이다의 IP주소와 현재 동작수직범위를 읽어온다.
fwSetYrlIpAddress 는 라이다의 IP주소를 변경한다.

- fwSetYrlVerticalAngles 는 라이다의 동작수직범위를 변경한다.
해당 함수를 사용하면 라이다가 껐다가 켜지며 약 13초정도의 시간이 소요된다.
추후 업데이트 버전에서는 라이다가 껐다가 켜지는 것 없이 훨씬 짧은 시간에 동작수직범위를 변경할 수 있게 할 예정이다.

3) test_distance_reader.cpp
- 기본적인 YRL 시리즈의 ranging을 시험할 수 있는 예제이다.
수평각도 플러스 마이너스 5도, 수직각도 플러스 마이너스 5도 사이 범위의 감지되는 물체들의 거리값을 출력한다.

4) test_error_code.cpp
- 기본적인 YRL 시리즈의 에러코드를 시험할 수 있는 예제이다.
시작할 때 에러 H가 발생하며, 주기적으로 getErrorCode 함수를 통해 에러값을 받아오고 해당 에러가 발견될 시 로그를 프린트한다.

* ROS Melodic Driver 유의사항
1) 기본적인 내용은 https://github.com/yujinrobot/yujin_lidar 을 참조한다.



