#ifndef _LIDAR_LOCALIZATION_ICP_SVD_REGISTRATION_HPP_
#define _LIDAR_LOCALIZATION_ICP_SVD_REGISTRATION_HPP_

#include <ros/ros.h>
#include<Eigen/Core>

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pcl/common/transforms.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class ICP_SVD_Registration:public Registration_Interface
    {
        public:
            ICP_SVD_Registration();
            ~ICP_SVD_Registration();


            bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
            bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                           const Eigen::Matrix4f& predict_pose, 
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) override;  

            size_t getCorrespondence(const CloudData::CLOUD_PTR &input_source,
                                     std::vector<Eigen::Vector3f> &xs,
                                     std::vector<Eigen::Vector3f> &ys);
            void getTransform(const std::vector<Eigen::Vector3f> &xs,
                              const std::vector<Eigen::Vector3f> &ys,
                              Eigen::Matrix4f &transformation); 

            // Eigen::Matrix4f calculateTranslation(const CloudData::CLOUD_PTR& input_source);         
        private:
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
            CloudData::CLOUD_PTR cloud_tmp_;
            CloudData::CLOUD_PTR input_source_;

            int max_iter_;
            Eigen::Matrix4f transform_matrix_ = Eigen::Matrix4f::Identity();;


    };
}


#endif