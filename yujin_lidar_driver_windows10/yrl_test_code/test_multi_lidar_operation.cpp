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
#include <fstream>

int main()
{
    std::ofstream fout;
    fout.open("a.txt");

    std::cout << "========================================" << std::endl;
    std::cout << "           Test YRL driver          " << std::endl;
    std::cout << "========================================\n" << std::endl;

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



    std::cout << "\n\n--------------------------------------------- Lidar_No1 calibration ---------------------------------------------\n" << std::endl;
    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance1 = producing_func();
    instance1->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance1->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance1->setCalibrationFilePath("C:/Users/jykim/OneDrive/Desktop/lk.bin");


    std::cout << "\n\n--------------------------------------------- Lidar_No2 calibration ---------------------------------------------\n" << std::endl;
    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance2 = producing_func();
    instance2->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance2->setInputIpAddress("192.168.1.12");
    /// Set LiDAR's Calibration File Path
    instance2->setCalibrationFilePath("C:/Users/jykim/OneDrive/Desktop/lk.bin");



    std::cout << "\n\n--------------------------------------------- Lidar_No3 calibration ---------------------------------------------\n" << std::endl;
    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance3 = producing_func();
    instance3->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance3->setInputIpAddress("192.168.1.13");
    /// Set LiDAR's Calibration File Path
    instance3->setCalibrationFilePath("C:/Users/jykim/OneDrive/Desktop/lk.bin");


    std::cout << "\n\n--------------------------------------------- Lidar_No4 calibration ---------------------------------------------\n" << std::endl;
    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance4 = producing_func();
    instance4->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance4->setInputIpAddress("192.168.1.14");
    /// Set LiDAR's Calibration File Path
    instance4->setCalibrationFilePath("C:/Users/jykim/OneDrive/Desktop/lk.bin");

        std::cout << "\nData Out-----------------------------------------------------------------------------------------------\n" << std::endl;

    /// Making a finite loop
    int timer(0);
    int period(40);
    int total_loop(10000000 * period);

    /// Buffers for getting output data
    std::vector<float> buffer_x, buffer_y, buffer_z;
    std::vector<float> intensity1, range1, horizontal_a1, vertical_a1;
    std::vector<float> intensity2, range2, horizontal_a2, vertical_a2;
    std::vector<float> intensity3, range3, horizontal_a3, vertical_a3;
    std::vector<float> intensity4, range4, horizontal_a4, vertical_a4;

    while (true)
    {
        timer += period;

        instance1->getSphericalOutputsWithIntensity(intensity1, range1, horizontal_a1, vertical_a1);
        instance2->getSphericalOutputsWithIntensity(intensity2, range2, horizontal_a2, vertical_a2);
        instance3->getSphericalOutputsWithIntensity(intensity3, range3, horizontal_a3, vertical_a3);
        instance4->getSphericalOutputsWithIntensity(intensity4, range4, horizontal_a4, vertical_a4);

        int size_buffer1(static_cast<int>(range1.size()));
        int size_buffer2(static_cast<int>(range2.size()));
        int size_buffer3(static_cast<int>(range3.size()));
        int size_buffer4(static_cast<int>(range4.size()));

        std::cout << "data : " << size_buffer1 << " " << size_buffer2 << " " << size_buffer3 << " " << size_buffer4 << " " << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(period));
        if (timer == total_loop)
        {
            break;
        }
    }

    /// release the memory allocated explicitly
    destroying_func(instance1);
    ::FreeLibrary(handle_to_dll);
    fout.close();

    return 0;
}
