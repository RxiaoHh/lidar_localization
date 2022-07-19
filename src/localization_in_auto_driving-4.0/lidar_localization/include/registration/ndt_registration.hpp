#ifndef _LIDAR_LOCALTION_NDT_REGISTRATION_HPP_
#define _LIDAR_LOCALTION_NDT_REGISTRATION_HPP_

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class NDT_Registration:public Registration_Interface
    {
        public:
            NDT_Registration();
            ~NDT_Registration();

            bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
            bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                           const Eigen::Matrix4f& predict_pose, 
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) override;
                           
            private:
                pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
            private:
                int max_iter_ ;  
                double ndt_res_ ;
                double step_size_ ;
                double trans_eps_ ;
    };
}


#endif