#ifndef _LIDAR_LICALIZATION_REGISTRATION_NDT_OMP_H_
#define _LIDAR_LICALIZATION_REGISTRATION_NDT_OMP_H_

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include <pclomp/ndt_omp.h> //ndt_omp
#include <pcl/filters/voxel_grid.h>
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class NDT_OMP_Registation:public Registration_Interface
    {
        public:
            NDT_OMP_Registation();
            ~NDT_OMP_Registation();

            bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
            bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                           const Eigen::Matrix4f& predict_pose, 
                           CloudData::CLOUD_PTR& result_cloud_ptr,
                           Eigen::Matrix4f& result_pose) override;
        
        private:
            pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_omp_;
        private:
            int max_iter_ ;  
            double ndt_omp_res_ ;
            double step_size_ ;
            double trans_eps_ ;            

    };
}


#endif