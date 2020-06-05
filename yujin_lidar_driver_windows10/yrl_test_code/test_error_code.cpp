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

    /// Using factory function, create a new YRL_Library object
    YRL_Library* instance = producing_func();
    instance->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("C:/Users/jykim/lktest.bin");

    /// Making a finite loop
    int timer(0);
    int period(1000);
    int total_loop(1000 * period);

    /// Error code string
    std::string error_code_string("");
    std::istringstream error_stream;
    std::string error;
    std::vector <std::string> errors;

    ///creating an error H
    instance->fwSetYrlVerticalAngles(-150, 150);

    while (true)
    {
        timer += period;

        instance->getErrorCode(error_code_string);

        std::cout << "Error Code : " << error_code_string << std::endl;

        error_stream = std::istringstream(error_code_string);
        while (getline(error_stream, error, ';'))
        {
            errors.push_back(error);
        }

        int num_of_errors = errors.size();
        for (int i(0); i < num_of_errors; i++)
        {
            if (errors.at(i).compare("A") == 0)
            {
                std::cout << "ERROR A : Power error. Please contact Yujin customer service for all support requests. " << std::endl;
            }
            if (errors.at(i).compare("B") == 0)
            {
                std::cout << "ERROR B : Horizontal movement error. Please contact Yujin customer service for all support requests. " << std::endl;
            }
            if (errors.at(i).compare("C") == 0)
            {
                std::cout << "ERROR C : Vertical movement error. Please contact Yujin customer service for all support requests. " << std::endl;
            }
            if (errors.at(i).compare("D") == 0)
            {
                std::cout << "ERROR D : Temperature measurement error. Please contact Yujin customer service for all support requests. " << std::endl;
            }
            if (errors.at(i).compare("E") == 0)
            {
                std::cout << "ERROR E : Sensor communication error. Please contact Yujin customer service for all support requests. " << std::endl;
            }
            if (errors.at(i).compare("F") == 0)
            {
                std::cout << "ERROR F : Operating temperature error. Please check the temperature of the operating environment, which should between -10 and 50 degree celsius. " << std::endl;
            }
            if (errors.at(i).compare("G") == 0)
            {
                std::cout << "ERROR G : User communication error. " << std::endl;
            }
            if (errors.at(i).compare("H") == 0)
            {
                std::cout << "ERROR H : Parameter data error. Please check the parameters of firmware interface functions. " << std::endl;
            }
            if (errors.at(i).compare("I") == 0)
            {
                std::cout << "ERROR I : Invalid command/code error. " << std::endl;
            }
        }
        errors.clear();

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
