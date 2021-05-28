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

#include "../../include/yrl_library/yrl_library.hpp"
#include <iostream>
#include <dlfcn.h>
//#include <thread>

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

    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance = producing_func();
    /// Correct Input Ip addres to driver is essential to use fw interface.
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("/home/jykim/lktest.bin");
    instance->start();

    /// fw interface
    int ip1(0), ip2(0), ip3(0), ip4(0);
    instance->fwGetYrlIpAddress(ip1, ip2, ip3, ip4);
    std::cout << "Current IP Address of LiDAR : " << ip1 << "." << ip2 << "." << ip3 << "." << ip4 << std::endl;

    int low(0), up(0);
    instance->fwGetYrlVerticalAngles(low, up);
    std::cout << "Current LiDAR Vertical Range : " << low << " to " << up << " in degrees" << std::endl;

    /// Setting new vertical angles
    instance->fwSetYrlVerticalAngles(-30, 30);

    instance->fwGetYrlVerticalAngles(low, up);
    std::cout << "New LiDAR Vertical Range : " << low << " to " << up << " in degrees" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    /// release the memory allocated explicitly

    destroying_func(instance);

    dlclose(handle_to_dynamiclib);

    return 0;
}
