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
#include <iostream>
#include "yrl_library.h"

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
    HINSTANCE handle_to_dll = ::LoadLibrary(TEXT("yrl_library.dll"));
    if (!handle_to_dll)
    {
        std::cerr << "\nCannot load DLL\n";
        return 1;
    }

    /// Get two functions from the library
    yrl_producer producing_func = reinterpret_cast<yrl_producer>(::GetProcAddress(handle_to_dll, "createYrlLib"));
    if (!producing_func)
    {
        std::cerr << "\nCannot get YRL_Library::createYrlLib from DLL\n";
        ::FreeLibrary(handle_to_dll);
        return 1;
    }

    yrl_destroyer destroying_func = reinterpret_cast<yrl_destroyer>(::GetProcAddress(handle_to_dll, "destroyYrlLib"));
    if (!destroying_func)
    {
        std::cerr << "\nCannot get YRL_Library::destroyYrlLib from DLL\n";
        ::FreeLibrary(handle_to_dll);
        return 1;
    }

    /// Using factory function, make a new YRL_Library object
    YRL_Library* instance = producing_func();
    instance->start();

    /// Set LiDAR's IP address as an input IP address. (Default is : 192.168.1.250)
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("C:/Users/jykim/lktest.bin");

    /// Creating a finite loop
    int timer(0);
    int period(10);
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
    ::FreeLibrary(handle_to_dll);

    return 0;
}
