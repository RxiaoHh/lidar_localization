#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_registration.hpp"

namespace lidar_localization
{
    ICP_Registration::ICP_Registration():icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>())
    {
        // std::string fin = "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/config/config.yaml";
        // YAML::Node yamlConfig = YAML::LoadFile(fin);
        // std::string registration_method = yamlConfig["registration_method"].as<std::string>();
        // double ndt_res_ = yamlConfig[registration_method]["res"].as<double>();
        // double step_size_ = yamlConfig[registration_method]["step_size"].as<double>();
        // double trans_eps_ = yamlConfig[registration_method]["trans_eps"].as<double>();
        // double max_iter_ = yamlConfig[registration_method]["max_iter"].as<double>();

        max_iter_ = 30;
        euc_fit_ = 0.1;
        trans_eps_ = 0.01;
        max_dis_ = 1.0;

        icp_ptr_->setMaxCorrespondenceDistance(max_dis_);
	    icp_ptr_->setMaximumIterations(max_iter_);
	    icp_ptr_->setTransformationEpsilon(trans_eps_);
	    icp_ptr_->setEuclideanFitnessEpsilon(euc_fit_);
   
        std::cout << "euc_fitness_: " << euc_fit_ << std::endl;
        std::cout << "max_dis_: " << max_dis_ << std::endl;
        std::cout << "trans_epsilon: " << trans_eps_ << std::endl;
        std::cout << "max_iter: " << max_iter_ << std::endl;        
    }

    ICP_Registration::~ICP_Registration()
    {     
    }

    bool ICP_Registration::SetInputTraget(CloudData::CLOUD_PTR& cloud_traget)
    {
        icp_ptr_->setInputTarget(cloud_traget);
        return true;
    }

    bool ICP_Registration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                     const Eigen::Matrix4f& predict_pose, 
                                     CloudData::CLOUD_PTR& result_cloud_ptr,
                                     Eigen::Matrix4f& result_pose) 
    {
        icp_ptr_->setInputSource(input_source);
        icp_ptr_->align(*result_cloud_ptr,predict_pose);
        result_pose = icp_ptr_->getFinalTransformation();

        return true;
    }

}
