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
#include "yrl_library.h" 

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "           Test YRL driver          " << std::endl;
    std::cout << "========================================\n" << std::endl;

    /// Load the dynamic library 
    HINSTANCE handle_to_dll = ::LoadLibrary(TEXT("YRL_Library.dll"));
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

    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance = producing_func();
    /// Set LiDAR's IP address as an input IP address for driver
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("C:/Users/userdirectory/lk_test.bin");

    instance->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /// fw interface
    int ip1(0), ip2(0), ip3(0), ip4(0);
    instance->fwGetYrlIpAddress(ip1, ip2, ip3, ip4);
    std::cout << "Current IP Address of LiDAR : " << ip1 << "." << ip2 << "." << ip3 << "." << ip4 << std::endl;

    int low(0), up(0);
    instance->fwGetYrlVerticalAngles(low, up);
    std::cout << "Current LiDAR Vertical Range : " << low << " to " << up << " in degrees" << std::endl;

    /// Setting new vertical angles
    instance->fwSetYrlVerticalAngles(-45, 45);

    instance->fwGetYrlVerticalAngles(low, up);
    std::cout << "New LiDAR Vertical Range : " << low << " to " << up << " in degrees" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /// release the memory allocated explicitly
    destroying_func(instance);
    ::FreeLibrary(handle_to_dll);

    return 0;
}