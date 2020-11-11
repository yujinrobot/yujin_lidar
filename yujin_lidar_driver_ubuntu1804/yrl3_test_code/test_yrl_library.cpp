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

#include "../../include/yrl_library/yrl_library.hpp"
#include <iostream>
#include <dlfcn.h>

int main()
{
    std::cout << "========================================" << std::endl; 
    std::cout << "           Test YRL driver          " << std::endl; 
    std::cout << "========================================\n" << std::endl;

    /// Load the dynamic library
    void* handle_to_dynamiclib = dlopen("libyrl_library.so",RTLD_LAZY);
    if(!handle_to_dynamiclib)
    {
        std::cerr<<"Cannot load shared library: "<<dlerror()<<'\n';
        return 1;
    }
    
    ///reset error
    dlerror();

    /// Get two functions from the library
    yrl_producer* producing_func = (yrl_producer*) dlsym(handle_to_dynamiclib,"createYrlLib");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr<<"Cannot get YRL_Library::createYrlLib from the library : "<<dlsym_error<<'\n';
        dlclose(handle_to_dynamiclib);
        return 1;
    }
    
    yrl_destroyer* destroying_func = (yrl_destroyer*) dlsym(handle_to_dynamiclib, "destroyYrlLib");
    dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr<<"Cannot get YRL_Library::destroyYrlLib from the library : "<<dlsym_error<<'\n';
        dlclose(handle_to_dynamiclib);
        return 1;
    }

    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance = producing_func();
    instance->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("/home/jykim/lktest.bin");

    /// Simple parameter get functions
    std::string inputIpAddress;
    instance->getInputIpAddress(inputIpAddress);
    std::cout << "inputIpAddress : " << inputIpAddress << std::endl;

    float sensor_height;
    instance->getSensorHeight(sensor_height);
    std::cout << "sensor_height : " << sensor_height << std::endl;

    double max_range;
    instance->getMaxRange(max_range);
    std::cout << "max_range : " << max_range << std::endl;

    float upper_data_limit;
    instance->getUpperDataLimit(upper_data_limit);
    std::cout << "upper_data_limit : " << upper_data_limit << std::endl;

    float lower_data_limit;
    instance->getLowerDataLimit(lower_data_limit);
    std::cout << "lower_data_limit : " << lower_data_limit << std::endl;

    double max_vertical_ang;
    instance->getMaxVerticalAngle(max_vertical_ang);
    std::cout << "max_vertical_ang : " << max_vertical_ang << std::endl;

    double min_vertical_ang;
    instance->getMinVerticalAngle(min_vertical_ang);
    std::cout << "min_vertical_ang : " << min_vertical_ang << std::endl;

    double max_horizontal_ang;
    instance->getMaxHorizontalAngle(max_horizontal_ang);
    std::cout << "max_horizontal_ang : " << max_horizontal_ang << std::endl;

    double min_horizontal_ang;
    instance->getMinHorizontalAngle(min_horizontal_ang);
    std::cout << "min_horizontal_ang : " << min_horizontal_ang << std::endl;

    float filter_level;
    instance->getCurrentFilterLevel(filter_level);
    std::cout << "filter_level : " << filter_level << std::endl;

    bool connection_state;
    instance->getConnectionState(connection_state);
    std::cout << "connection_state : " << connection_state << std::endl;

    double rotation_per_sec;

    /// Simple parameter set functions
    instance->setSensorHeight(1.5); /// sensor at 1.5m height. The default height when the sensor is on ground is 0.07m
    instance->setMaxRange(30); /// max range of 30m
    instance->setUpperDataLimit(4); /// Data upper limit of 4 when the sensor is at 1.5m height
    instance->setLowerDataLimit(-2); /// Data lower limit of -2 when the sensor is at 1.5m height
    instance->setMaxVerticalAngle(45); /// default value of 45 (total 90 degrees of vertical FOV)
    instance->setMinVerticalAngle(-45); /// default value of -45 (total 90 degrees of vertical FOV)
    instance->setMaxHorizontalAngle(135); /// default value of 135 (total 270 degrees of horizontal FOV)
    instance->setMinHorizontalAngle(-135); /// default value of -135 (total 270 degrees of horizontal FOV)
    instance->setCurrentFilterLevel(0.01); /// Default filter level is 0.01. Depending on user applications, this filter level may need to be adjusted.

    std::cout << "\nAfter Setting-----------------------------------------------------------------------------------------------\n" << std::endl;

    /// Making a finite loop
    int timer(0);
    int period(40);
    int total_loop(10000000 * period);

    /// Buffers for getting output data
    std::vector<float> buffer_x, buffer_y, buffer_z;
    std::vector<float> intensity, range, horizontal_a, vertical_a;

    while (true)
    {
        timer += period;

        instance->getRPS(rotation_per_sec);
        std::cout << "rotation_per_sec : " << rotation_per_sec << std::endl;

        instance->getCartesianOutputs(buffer_x, buffer_y, buffer_z);
        int size_buffer(static_cast<int>(buffer_x.size()));
        for (int i(0); i < size_buffer; i++)
        {
            std::cout << "Coordinate(" << i << ") : (" << buffer_x.at(i) << "," << buffer_y.at(i) << "," << buffer_z.at(i) << ")" << std::endl;
        }

        instance->getSphericalOutputsWithIntensity(intensity, range, horizontal_a, vertical_a);
        int size_buffer2(static_cast<int>(range.size()));
        for (int j(0); j < size_buffer2; j++)
        {
            std::cout << "intensity(" << j << ") : " << intensity.at(j) << ", " << "range(" << j << ") : " << range.at(j) << ", " << "HA(" << j << ") : " << horizontal_a.at(j) << ", " << "VA(" << j << ") : " << vertical_a.at(j) << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(period));
        if (timer == total_loop)
        {
            break;
        }
    }

    /// release the memory allocated explicitly
    destroying_func(instance);
    dlclose(handle_to_dynamiclib);

    return 0;
}
