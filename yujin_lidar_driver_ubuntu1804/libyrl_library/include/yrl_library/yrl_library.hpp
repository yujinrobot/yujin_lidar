/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*
*  Ju Young Kim, jykim3@yujinrobot.com
*
*  Non-monifiable freely redistributable software(FRS)
*
*  - Redistribution. Redistribution and use in binary form, without modification,
*    are permitted provided that the following conditions are met:
*  - Redistributions must reproduce the above copyright notice and the following
*    disclaimer in the documentation and/or other materials provided with the distribution.
*  - Neither the name of YujinRobot Corporation nor the names of its suppliers may be used
*    to endorse or promote products derived from this software without specific prior written permission.
*
*  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OFOR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*********************************************************************/

#ifndef YRL_LIBRARY_HPP_
#define YRL_LIBRARY_HPP_

#if defined _WIN32 || defined __CYGWIN__
#ifdef YRL_LIBRARY_EXPORTS
#ifdef __GNUC__
#define YRL_LIBRARYSHARED_EXPORT __attribute__ ((dllexport))
#else
#define YRL_LIBRARYSHARED_EXPORT __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define YRL_LIBRARYSHARED_EXPORT __attribute__ ((dllimport))
#else
#define YRL_LIBRARYSHARED_EXPORT __declspec(dllimport)
#endif
#endif
#define YRL_LIBRARYSHARED_NO_EXPORT
#else
#if __GNUC__ >= 4
#define YRL_LIBRARYSHARED_EXPORT __attribute__ ((visibility ("default")))
#define YRL_LIBRARYSHARED_NO_EXPORT  __attribute__ ((visibility ("hidden")))
#else
#define YRL_LIBRARYSHARED_EXPORT
#define YRL_LIBRARYSHARED_NO_EXPORT
#endif
#endif

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

#define NO_BYTES_FOR_HEADER_ID      4
#define NO_BYTES_FOR_FW_VERSION     2
#define NO_BYTES_FOR_HW_VERSION     1
#define NO_BYTES_FOR_TIMESTAMP      4
#define NO_BYTES_FOR_DES            1
#define NO_BYTES_FOR_RESERVED       4
#define NO_BYTES_FOR_ALL (NO_BYTES_FOR_HEADER_ID + NO_BYTES_FOR_FW_VERSION + NO_BYTES_FOR_HW_VERSION + NO_BYTES_FOR_TIMESTAMP + NO_BYTES_FOR_DES + NO_BYTES_FOR_RESERVED)

#define IDX_FW_VERSION   (             0 + NO_BYTES_FOR_HEADER_ID) //4
#define IDX_HW_VERSION   (IDX_FW_VERSION + NO_BYTES_FOR_FW_VERSION) //6
#define IDX_TIME_STAMP   (IDX_HW_VERSION + NO_BYTES_FOR_HW_VERSION) //7
#define IDX_DES          (IDX_TIME_STAMP + NO_BYTES_FOR_TIMESTAMP) //11
#define IDX_RESERVED     (IDX_DES               + NO_BYTES_FOR_DES) //12

#define IDX_SUB_HEAD     (IDX_DES + 1) //12
#define IDX_SUB_LENGTH   (IDX_SUB_HEAD + 2) //14

#define MASTER_DES_SIGNAUTRE0           0x02
#define MASTER_DES_SIGNAUTRE1           0x02
#define MASTER_DES_SIGNAUTRE2           0x02
#define MASTER_DES_SIGNAUTRE3           0x02

#define MASTER_DES_PARAMETERS                       0
#define MASTER_DES_COMMANDS                     1
#define MASTER_DES_FEEDBACKS                    2
#define MASTER_DES_REQUEST_PARAMETERS       3
#define MASTER_DES_UPDATED_PARAMETERS       4

#include <cstring>
#include <fcntl.h>
#include <vector>
#include <map>
#include <deque>
#include <limits>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma warning(disable:4244) 
#else
#include <sys/socket.h>
//#include <sys/types.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
//#include <netinet/in.h>   
#endif

class YRL_Library
{
public:
    virtual ~YRL_Library();
    virtual int start() = 0;
protected:
    virtual void stop() = 0;
    virtual unsigned int _3bytesToUint(unsigned char* ptr) = 0;
    virtual void appendData(unsigned char* ptr, const int noData) = 0;
    virtual void proc() = 0;
public:
    virtual void run() = 0; ///Do not use this run function.

    ///Parameter Read Interface
    virtual void getErrorCode(std::string& error_code) = 0;
    virtual void getInputIpAddress(std::string& ip_address) = 0;
    virtual void getSensorHeight(float& sensor_height) = 0; /// Only for 3D
    virtual void getMaxRange(double& max_range) = 0;
    virtual void getUpperDataLimit(float& upper_data_limit) = 0; /// Only for 3D
    virtual void getLowerDataLimit(float& lower_data_limit) = 0; /// Only for 3D
    virtual void getMaxVerticalAngle(double& max_vertical_ang) = 0; /// Only for 3D
    virtual void getMinVerticalAngle(double& min_vertical_ang) = 0; /// Only for 3D
    virtual void getMaxHorizontalAngle(double& max_horizontal_ang) = 0;
    virtual void getMinHorizontalAngle(double& min_horizontal_ang) = 0;
    virtual void getCurrentFilterLevel(float& filter_level) = 0; /// Only for 3D

    ///Parameter Write Interface
    virtual void setCalibrationFilePath(std::string cal_file_path) = 0;
    virtual void setInputIpAddress(std::string ip_address) = 0;
    virtual void setSensorHeight(float sensor_height) = 0; /// Only for 3D
    virtual void setMaxRange(double max_range) = 0;
    virtual void setUpperDataLimit(float upper_data_limit) = 0; /// Only for 3D 
    virtual void setLowerDataLimit(float lower_data_limit) = 0; /// Only for 3D
    virtual void setMaxVerticalAngle(double max_vertical_ang) = 0; /// Only for 3D
    virtual void setMinVerticalAngle(double min_vertical_ang) = 0; /// Only for 3D
    virtual void setMaxHorizontalAngle(double max_horizontal_ang) = 0;
    virtual void setMinHorizontalAngle(double min_horizontal_ang) = 0;
    virtual void setCurrentFilterLevel(float filter_level) = 0; /// Only for 3D

    ///Output Read Interface
    /*
    virtual int getThreadCount() = 0;
    virtual int getSuccessfulCommCount() = 0;
    virtual int getDataCount() = 0;
    virtual int getDprCount() = 0;
    */
    virtual void getConnectionState(bool& connection_state) = 0;
    virtual void getRPS(double& rotation_per_sec) = 0;
    virtual void getCartesianOutputs(std::vector <float>& output_x, std::vector <float>& output_y, std::vector <float>& output_z) = 0;
    virtual void getCartesianOutputsWithIntensity(std::vector <float>& intensity, std::vector <float>& output_x, std::vector <float>& output_y, std::vector <float>& output_z) = 0;
    virtual void getSphericalOutputs(std::vector <float>& range, std::vector <float>& horizontal_ang, std::vector <float>& vertical_ang) = 0;
    virtual void getSphericalOutputsWithIntensity(std::vector <float>& intensity, std::vector <float>& range, std::vector <float>& horizontal_ang, std::vector <float>& vertical_ang) = 0;

    ///LiDAR Setting Interface
    virtual void fwGetYrlIpAddress(int& ip_a, int& ip_b, int& ip_c, int& ip_d) = 0;
    virtual void fwSetYrlIpAddress(int ip_a, int ip_b, int ip_c, int ip_d) = 0;
    virtual void fwGetYrlVerticalAngles(int& lower_angle, int& upper_angle) = 0; /// Only for 3D
    virtual void fwSetYrlVerticalAngles(int lower_angle, int upper_angle) = 0; /// Only for 3D
    virtual void fwGetModelNo(unsigned int& model_no) = 0;
};


#ifdef __GNUC__
typedef int SOCKET;
typedef YRL_Library* yrl_producer();
typedef void yrl_destroyer(YRL_Library*);
#endif

#ifdef _MSC_VER
typedef YRL_Library* (__cdecl* yrl_producer)();
typedef void(__cdecl* yrl_destroyer) (YRL_Library*);
#endif

extern "C" YRL_LIBRARYSHARED_EXPORT YRL_Library * createYrlLib();
extern "C" YRL_LIBRARYSHARED_EXPORT void destroyYrlLib(YRL_Library * lib);
#endif // YRL_LIBRARY_HPP_