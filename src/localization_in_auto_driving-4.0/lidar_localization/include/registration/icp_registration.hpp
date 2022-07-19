#ifndef _LIDAR_LOCALLIZATION_ICP_REGISTRATION_HPP_
#define _LIDAR_LOCALLIZATION_ICP_REGISTRATION_HPP_

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class ICP_Registration:public Registration_Interface
    {
        public:
            ICP_Registration();
            ~ICP_Registration();

            bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
            bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                           const Eigen::Matrix4f& predict_pose, 
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) override;
        private:
            pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;

        private:
            int max_iter_ ;  
            double max_dis_ ;
            double euc_fit_ ;
            double trans_eps_ ;            
    };
}


#endif


