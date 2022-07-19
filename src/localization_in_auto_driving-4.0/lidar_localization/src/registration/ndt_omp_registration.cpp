#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/ndt_omp_registration.hpp"

namespace lidar_localization
{
    NDT_OMP_Registation::NDT_OMP_Registation()
    :ndt_omp_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        // std::string fin = "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/config/config.yaml";
        // YAML::Node yamlConfig = YAML::LoadFile(fin);
        // std::string registration_method = yamlConfig["registration_method"].as<std::string>();
        // double ndt_omp_res_ = yamlConfig[registration_method]["res"].as<double>();
        // double step_size_ = yamlConfig[registration_method]["step_size"].as<double>();
        // double trans_eps_ = yamlConfig[registration_method]["trans_eps"].as<double>();
        // double max_iter_ = yamlConfig[registration_method]["max_iter"].as<double>();

        ndt_omp_res_ = 1.0;
        trans_eps_ = 0.01;
        step_size_ = 0.1;
        max_iter_ = 30;

        ndt_omp_->setResolution(ndt_omp_res_);
        ndt_omp_->setNumThreads(4);
        ndt_omp_->setNeighborhoodSearchMethod(pclomp::KDTREE);
        ndt_omp_->setTransformationEpsilon(trans_eps_); 
        ndt_omp_->setStepSize(step_size_);
        ndt_omp_->setMaximumIterations(max_iter_); 
    
        std::cout << "ndt_omp_res_: " << ndt_omp_res_ << std::endl;
        std::cout << "step_size: " << step_size_ << std::endl;
        std::cout << "trans_epsilon: " << trans_eps_ << std::endl;
        std::cout << "max_iter: " << max_iter_ << std::endl;   

    }
    NDT_OMP_Registation::~NDT_OMP_Registation()
    {
    }

    bool NDT_OMP_Registation::SetInputTraget(CloudData::CLOUD_PTR& cloud_traget)
    {
        ndt_omp_->setInputTarget(cloud_traget);
        return true;
    }

    bool NDT_OMP_Registation::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                        const Eigen::Matrix4f& predict_pose, 
                                        CloudData::CLOUD_PTR& result_cloud_ptr,
                                        Eigen::Matrix4f& result_pose)
    {
        ndt_omp_->setInputSource(input_source);
        ndt_omp_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndt_omp_->getFinalTransformation(); 

        return true;
    }
}
