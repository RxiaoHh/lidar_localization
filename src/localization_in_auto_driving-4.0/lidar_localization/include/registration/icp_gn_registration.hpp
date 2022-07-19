#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_MANUAL_HPP_

#include <pcl/registration/icp.h>    
#include <pcl/kdtree/kdtree_flann.h>
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/third_party/sophus/sophus/se3.hpp" 
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization 
{
    class ICP_GN_Registration: public Registration_Interface 
    {
        public:
            ICP_GN_Registration();

            bool SetInputTraget(CloudData::CLOUD_PTR& input_target) override;
            bool ScanMatch(const CloudData::CLOUD_PTR   &input_source, 
                           const Eigen::Matrix4f    &predict_pose, 
                           CloudData::CLOUD_PTR   &result_cloud_ptr,
                           Eigen::Matrix4f   &result_pose) override;
    
        private:
            void calculateTrans(const CloudData::CLOUD_PTR &input_cloud );// 计算旋转矩阵

        private:
            CloudData::CLOUD_PTR target_cloud_;        
            pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;
            float max_correspond_distance_;      
            int max_iterator_;                                

            Eigen::Matrix3f  rotation_matrix_;       //旋转矩阵
            Eigen::Vector3f  translation_;           //平移矩阵
            Eigen::Matrix4f  transformation_;        //转换矩阵     
        
    };
}

#endif


