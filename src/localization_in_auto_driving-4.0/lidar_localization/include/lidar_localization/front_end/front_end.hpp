#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>//ndt
#include <pclomp/ndt_omp.h> //ndt_omp

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/ndt_registration.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/ndt_omp_registration.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_registration.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_svd_registration.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_ceres_registration.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_gn_registration.hpp"
// #include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/opt_ICP_CERES.h"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/lidarFactor.hpp"

namespace lidar_localization 
{
  class FrontEnd 
  {
    public:
      class Frame 
      {
        public:  
          Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
          CloudData cloud_data;
      };

    public:
      // FrontEnd();
      FrontEnd(ros::NodeHandle nh);

      Eigen::Matrix4f Update(const CloudData& cloud_data);       
      bool SetInitPose(const Eigen::Matrix4f& init_pose);
      bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

      bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
      bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
      bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);
    
    private:
      void UpdateNewFrame(const Frame& new_key_frame);

    private:
      pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
      pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
      pcl::VoxelGrid<CloudData::POINT> display_filter_;
      pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
      pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_omp_;

      std::deque<Frame> local_map_frames_;
      std::deque<Frame> global_map_frames_;

      bool has_new_local_map_ = false;
      bool has_new_global_map_ = false;
      CloudData::CLOUD_PTR local_map_ptr_;
      CloudData::CLOUD_PTR global_map_ptr_;
      CloudData::CLOUD_PTR result_cloud_ptr_;
      Frame current_frame_;

      Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();

    //配置参数赋值
    private:
      ros::NodeHandle nh_;
      int max_iter_ ;  
      double ndt_res_ ;
      double step_size_ ;
      double trans_eps_ ;

      double cloud_set_leaf_size_;
      double local_set_leaf_size_;
      double dis_set_leaf_size_;

      std::shared_ptr<Registration_Interface> registration_ptr_; 
      
  };
}

#endif