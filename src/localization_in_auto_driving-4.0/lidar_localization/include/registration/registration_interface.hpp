#ifndef _LIDAR_LOCALTION_REGISTRATION_INTERFACE_HPP_
#define _LIDAR_LOCALTION_REGISTRATION_INTERFACE_HPP_

#include <ros/ros.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization
{
    class Registration_Interface
    {
        public:
            Registration_Interface() {};
            ~Registration_Interface() {};

            virtual bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) = 0;
            virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                   const Eigen::Matrix4f& predict_pose, 
                                   CloudData::CLOUD_PTR& result_cloud_ptr,
                                   Eigen::Matrix4f& result_pose) = 0;
            
    };
}



#endif