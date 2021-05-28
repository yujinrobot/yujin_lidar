/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*  
*  Hyeon Jeong Kim, hjkim2@yujinrobot.com
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

#include <iostream>
#include <dlfcn.h>
#include "../../include/yrl_library/yrl_library.hpp"

/// Function that converts radians to degrees
float radianTodegree(float radian)
{
    return (radian * 180.0f) / 3.14159265358979;
}

/// Function that returns bool flag whether an 'object' is within plus minus 'positive_value' range
bool rangeIn2D(float& object, float positive_value)
{
    if (object < positive_value)
    {
        if (object > -positive_value)
        {
            return true;
        }
    }
    return false;
}

int main()
{
    std::cout << "=======================================" << std::endl;
    std::cout << "Test YRL driver                       " << std::endl;
    std::cout << "=======================================\n" << std::endl;

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

    /// Using factory function, make a new YRL_Library object
    YRL_Library* instance = producing_func();
    /// Set LiDAR's IP address as an input IP address. (Default is : 192.168.1.250)
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("/home/jykim/lktest.bin");
    instance->start();

    /// Creating a finite loop
    int timer(0);
    int period(40);
    int total_loop(1000 * period);

    /// Buffers to get output data
    std::vector <float> range, ha, va;
    /// Temporary horizontal and vertical values
    float tmp_h(0);
    float tmp_v(0);
    /// Flags that show whether the value is in the targeting range
    bool hr(true);
    bool vr(true);

    while (true)
    {
        timer += period;

        instance->getSphericalOutputs(range, ha, va);

        int num = range.size();
        int degree_range = 5;
        for (int i(0); i < num; i++)
        {
            tmp_h = radianTodegree(ha.at(i));
            tmp_v = radianTodegree(va.at(i));
            hr = rangeIn2D(tmp_h, degree_range);
            vr = rangeIn2D(tmp_v, degree_range);
            if (hr && vr)
            {
                std::cout << "Front measurement : " << range.at(i) << "m" << std::endl;
            }
        }
        std::cout << "WAITING FOR MORE READS" << std::endl;

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
