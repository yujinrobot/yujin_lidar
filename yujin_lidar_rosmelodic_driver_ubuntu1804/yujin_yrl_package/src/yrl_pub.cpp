/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*  
*  Hyeon Jeong Kim, hjkim2@yujinrobot.com
*
*  Software License Agreement (BSD License)
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  - Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above
*  copyright notice, this list of conditions and the following
*  disclaimer in the documentation and/or other materials provided
*  with the distribution.
*  - Neither the name of {copyright_holder} nor the names of its
*  contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/// This ROS Driver import YRL Linux Driver
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
*  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIESWITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OFOR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*********************************************************************/

#include "yrl_library.hpp"
#include <dlfcn.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv)
{
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
    /// Set LiDAR's IP address as an input IP address for driver
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("/home/yourdirectory/lk_test.bin");
    instance->start();
    
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
    
    /// initialize a node
    ros::init(argc, argv, "yrl_pub");
    
    /// NodeHandle
    ros::NodeHandle nh("~");

    /// Publisher yrl_pub
    ros::Publisher yrl_pub = nh.advertise<sensor_msgs::PointCloud2>("yrl_cloud", 2);
	const uint32_t point_size = 16;

    /// tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation(q);

    /// Buffers for getting output data
    std::vector<float> buffer_x, buffer_y, buffer_z, buffer_i;

    /// Error code string
    std::string error_code_string("");
    std::istringstream error_stream;
    std::string error;
    std::vector <std::string> errors;

    ros::Rate loop_rate(22); ///22Hz loop rate
    
    while(ros::ok())
    {
        instance->getCartesianOutputsWithIntensity(buffer_i, buffer_x, buffer_y, buffer_z);
        int size_buffer(static_cast<int>(buffer_x.size()));     

        /// Publish a group of point clouds
		sensor_msgs::PointCloud2 cloud;
		cloud.header.frame_id = std::string("yrl_cloud_id");
        cloud.header.stamp = ros::Time::now();

        cloud.fields.resize(4);
		cloud.fields[0].name = "x";
		cloud.fields[0].offset = 0;
		cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[0].count = 1;

		cloud.fields[1].name = "y";
		cloud.fields[1].offset = 4;
		cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[1].count = 1;

		cloud.fields[2].name = "z";
		cloud.fields[2].offset = 8;
		cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[2].count = 1;

		cloud.fields[3].name = "intensity";
		cloud.fields[3].offset = 12;
		cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
		cloud.fields[3].count = 1;

		cloud.data.resize(std::max(1, size_buffer) * point_size, 0x00);
		cloud.point_step = point_size;
		cloud.row_step = cloud.data.size();
		cloud.height = 1;
		cloud.width = cloud.row_step / point_size;
		cloud.is_bigendian = false;
		cloud.is_dense = true;
	  
	    uint8_t *ptr = cloud.data.data();
		for (int i(0); i < size_buffer; i++)
		{
			*(reinterpret_cast<float*>(ptr +  0)) = buffer_x[i];
			*(reinterpret_cast<float*>(ptr +  4)) = buffer_y[i];
			*(reinterpret_cast<float*>(ptr +  8)) = buffer_z[i];
			*(reinterpret_cast<float*>(ptr + 12)) = buffer_i[i];
			ptr += point_size;
		}

        yrl_pub.publish(cloud);
        buffer_x.clear();
        buffer_y.clear();
        buffer_z.clear();
        buffer_i.clear();

        /// Send Transform
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "yrl_cloud_id"));
        
        /// Error Check
        instance->getErrorCode(error_code_string);       
//         std::cout<<"Error Code : "<<error_code_string<<std::endl;
        
        error_stream = std::istringstream(error_code_string);
        while(getline(error_stream, error, ';'))
        {
            errors.push_back(error);
        }
        
        int num_of_errors = errors.size();
        for(int i(0);i<num_of_errors;i++)
        {
                if(errors.at(i).compare("A") == 0)
                {
                    std::cout<<"ERROR A : Power error. Please contact Yujin customer service for all support requests. "<<std::endl;
                }
                if(errors.at(i).compare("B") == 0)
                {
                    std::cout<<"ERROR B : Horizontal movement error. Please contact Yujin customer service for all support requests. "<<std::endl;
                }
                if(errors.at(i).compare("C") == 0)
                {
                    std::cout<<"ERROR C : Vertical movement error. Please contact Yujin customer service for all support requests. "<<std::endl;
                }
                if(errors.at(i).compare("D") == 0)
                {
                    std::cout<<"ERROR D : Temperature measurement error. Please contact Yujin customer service for all support requests. "<<std::endl;
                }
                if(errors.at(i).compare("E") == 0)
                {
                    std::cout<<"ERROR E : Data communication error. Please contact Yujin customer service for all support requests. "<<std::endl;
                }
                if(errors.at(i).compare("F") == 0)
                {
                    std::cout<<"ERROR F : Operating temperature error. Please check the temperature of the operating environment, which should between -10 and 50 degree celsius. "<<std::endl;
                }
                if(errors.at(i).compare("G") == 0)
                {
                    std::cout<<"ERROR G : User communication error. Please check product's connection state. "<<std::endl;
                }
                if(errors.at(i).compare("H") == 0)
                {
                    std::cout<<"ERROR H : Parameter data error. Please check the parameters of firmware interface functions. "<<std::endl;
                }
                if(errors.at(i).compare("I") == 0)
                {
                    std::cout<<"ERROR I : Invalid command error. "<<std::endl;
                }
        }
        errors.clear();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    /// release the memory allocated explicitly
    destroying_func(instance);
    dlclose(handle_to_dynamiclib);
    
    return 0;
}
