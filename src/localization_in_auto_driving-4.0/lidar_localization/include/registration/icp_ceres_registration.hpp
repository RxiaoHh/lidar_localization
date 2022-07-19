#ifndef _LIDAR_LOCALIZATION_ICP_CERES_REGISTRATION_HPP_
#define _LIDAR_LOCALIZATION_ICP_CERES_REGISTRATION_HPP_

#include <cmath>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class ICP_CERES_Registrtion:public Registration_Interface
    {
        public:
            ICP_CERES_Registrtion();
            ~ICP_CERES_Registrtion();

            bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
            bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                           const Eigen::Matrix4f& predict_pose, 
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) override;              

        private:
            double para_q[4] = {0,0,0,1};
            double para_t[3] = {0,0,0};
            
            Eigen::Map<Eigen::Quaterniond> q_last_curr = Eigen::Map<Eigen::Quaterniond>(para_q);
            Eigen::Map<Eigen::Vector3d> t_last_curr = Eigen::Map<Eigen::Vector3d>(para_t);

            CloudData::CLOUD_PTR cloud_tmp_;
            CloudData::CLOUD_PTR input_source_;
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_; 
    };
}

#endif

