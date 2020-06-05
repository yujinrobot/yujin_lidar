/*********************************************************************
*  Copyright (c) 2020, YujinRobot Corp.
*  
*  Ju Young Kim, jykim3@yujinrobot.com
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
*  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIESWITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OFOR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*********************************************************************/

#include "yrl_library.hpp"
#include <dlfcn.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

void ValueToRgb(const float aValue, float &r, float &g, float &b)
{
    float value( aValue );
    if( value < 0.0f ) value = 0.0f;
    else if( value > 1.0f ) value = 1.0f;

    if( value < 0.25f )
    {
        r = 1.0f;
        g = value / 0.25f;
        b = 0.0f;
    }
    else if( value < 0.5f )
    {
        r = 1.0f - (value-0.25f) / 0.25f;
        g = 1.0f;
        b = 0.0f;
    }
    else if( value < 0.75f )
    {
        r = 0.0f;
        g = 1.0f;
        b = (value-0.5f)/0.25f;
    }
    else
    {
        r = 0.0f;
        g = 1.0f - (value-0.75f)/0.25f;
        b = 1.0f;
    }
}

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
    instance->start();

    /// Set LiDAR's IP address as an input IP address for driver
    instance->setInputIpAddress("192.168.1.250");
    /// Set LiDAR's Calibration File Path
    instance->setCalibrationFilePath("/home/jykim/catkin_ws/lktest.bin");
    
    /// Simple parameter set functions
    instance->setSensorHeight(1.5);
    instance->setUpperDataLimit(4);
    instance->setLowerDataLimit(0);
    instance->setMaxVerticalAngle(45);
    instance->setMinVerticalAngle(-45);
    instance->setMaxHorizontalAngle(135);
    instance->setMinHorizontalAngle(-135);
    instance->setCurrentFilterLevel(0.01);
    
    /// get upper_data_limit & lower_data_limit for coloring point clouds in rviz.
    float upper_data_limit;
    instance->getUpperDataLimit(upper_data_limit);
    float lower_data_limit;
    instance->getLowerDataLimit(lower_data_limit);
    
    /// initialize a node
    ros::init(argc, argv, "yrl_pub");
    
    /// NodeHandle
    ros::NodeHandle nh("~");

    /// Publisher yrl_pub
    ros::Publisher yrl_pub = nh.advertise<sensor_msgs::PointCloud>("yrl_cloud",100);

    /// Point Cloud Message
    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = std::string("yrl_cloud_id");
    cloud.header.stamp = ros::Time::now();
    /// Point Cloud Coordinate and Color
    geometry_msgs::Point32 point;
    sensor_msgs::ChannelFloat32 channel_r;
    sensor_msgs::ChannelFloat32 channel_g;
    sensor_msgs::ChannelFloat32 channel_b;    
    channel_r.name=std::string("r");
    channel_g.name=std::string("g");
    channel_b.name=std::string("b");
    cloud.channels.push_back(channel_r);
    cloud.channels.push_back(channel_g);
    cloud.channels.push_back(channel_b);
    
    /// tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation(q);

    /// Buffers for getting output data
    std::vector<float> buffer_x, buffer_y, buffer_z;
    std::vector<float> intensity, range, horizontal_a, vertical_a;
    float rr(0), gg(0), bb(0);

    /// Error code string
    std::string error_code_string("");
    std::istringstream error_stream;
    std::string error;
    std::vector <std::string> errors;

    ros::Rate loop_rate(60); ///previous value = 1000Hz
    
    while(ros::ok())
    {
        instance->getCartesianOutputs(buffer_x, buffer_y, buffer_z);
        int size_buffer(static_cast<int>(buffer_x.size()));
        
        for (int i(0); i < size_buffer; i++)
        {
            /// Getting coordinates of each point cloud
            point.x = buffer_x.at(i);
            point.y = buffer_y.at(i);
            point.z = buffer_z.at(i);
            cloud.points.push_back(point);
            
            /// Coloring each point cloud for 3D and 2D
            unsigned int model_no(0);
            instance->fwGetModelNo(model_no);
            if(model_no == 4 || model_no == 5 || model_no == 6) /// 2D
            {
                rr = 1.0f;
                gg = 1.0f;
                bb = 1.0f;
            }
            else
            {
                ///3D
                ValueToRgb(point.z/(upper_data_limit-lower_data_limit), rr, gg, bb);
            }
            
            cloud.channels[0].values.push_back(rr);
            cloud.channels[1].values.push_back(gg);
            cloud.channels[2].values.push_back(bb);            
        }

        /// Publish a group of point clouds
        cloud.header.stamp = ros::Time::now();
        yrl_pub.publish(cloud);
        
        /// Send Transform
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "yrl_cloud_id"));

        /// Clear buffers for point clouds
        cloud.points.clear();
        cloud.channels[0].values.clear();
        cloud.channels[1].values.clear();
        cloud.channels[2].values.clear();
        
        
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
