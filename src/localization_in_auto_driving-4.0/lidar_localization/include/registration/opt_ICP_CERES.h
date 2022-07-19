#pragma once
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/registration_interface.hpp"

namespace lidar_localization
{
    class opt_ICP_CERES : public Registration_Interface
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // opt_ICP_CERES(const YAML::Node &node);
        opt_ICP_CERES();
        ~opt_ICP_CERES();
        bool SetInputTraget(CloudData::CLOUD_PTR& cloud_traget) override;
        bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                   const Eigen::Matrix4f& predict_pose, 
                                   CloudData::CLOUD_PTR& result_cloud_ptr,
                                   Eigen::Matrix4f& result_pose) override;

        float getFitnessScore();

    private:
        CloudData::CLOUD_PTR target_ptr, source_ptr;
        Eigen::Matrix4f final_pose;
        int max_iterations;
        float max_coresspoind_dis;
        float trans_eps;
        float euc_fitness_eps;

        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
        Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

        pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_flann;
    };
}