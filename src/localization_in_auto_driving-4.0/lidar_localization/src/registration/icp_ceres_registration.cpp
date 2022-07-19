
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/lidarFactor.hpp"
#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_ceres_registration.hpp"

namespace lidar_localization
{
    Eigen::Quaterniond q_w_curr(1,0,0,0);//最终的位姿
    Eigen::Vector3d t_w_curr(0,0,0);

    ICP_CERES_Registrtion::ICP_CERES_Registrtion()
    :kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()),
     cloud_tmp_(new CloudData::CLOUD()),
     input_source_(new CloudData::CLOUD())
    {
    }

    ICP_CERES_Registrtion::~ICP_CERES_Registrtion()
    {
    }

    bool ICP_CERES_Registrtion::SetInputTraget(CloudData::CLOUD_PTR& cloud_traget)
    {
        cloud_tmp_ = cloud_traget;
        kdtree_->setInputCloud(cloud_traget);
        return true;
    }

    bool ICP_CERES_Registrtion::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                          const Eigen::Matrix4f& predict_pose, 
                                          CloudData::CLOUD_PTR& result_cloud_ptr,
                                          Eigen::Matrix4f& result_pose) 
    {
        input_source_ = input_source;
        CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
        pcl::transformPointCloud(*input_source, *transformed_input_source, predict_pose);//将当前帧转换到世界坐标系(就是转换到局部地图中)
                                                                                         //将当前帧和目标帧保持在同一坐标系下才能匹配
        int k = 1;
        size_t number_point = 0;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquareDistance(k);

        //构建寻优问题
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_q,4,q_parameterization);
        problem.AddParameterBlock(para_t,3);

        for(int i = 0;i < transformed_input_source->points.size();i++)  
        {
            kdtree_->nearestKSearch(transformed_input_source->at(i),k,pointIdxNKNSearch,pointNKNSquareDistance);
            if(pointNKNSquareDistance.at(0) > 1.0)
                continue;
            
            Eigen::Vector3d curr_point(transformed_input_source->at(i).x,
                                       transformed_input_source->at(i).y,
                                       transformed_input_source->at(i).z);
            Eigen::Vector3d last_point(cloud_tmp_->at(pointIdxNKNSearch[0]).x,     
                                       cloud_tmp_->at(pointIdxNKNSearch[0]).y,
                                       cloud_tmp_->at(pointIdxNKNSearch[0]).z);

            ceres::CostFunction *cost_function = ICP_Error::Create(curr_point,last_point);
            problem.AddResidualBlock(cost_function,loss_function,para_q,para_t);
        }

        //配置并运行求解器
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 40;
        options.minimizer_progress_to_stdout = false;//是否向终端输出优化信息
        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);

        //四元素代表的世界坐标
        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;  

        std::cout << "t_w_curr = " << t_w_curr << std::endl;

        //将四元素转化成变换矩阵
        Eigen::Matrix3f temp_transformation = q_w_curr.toRotationMatrix().cast<float>();
        result_pose.block<3,3>(0,0) = temp_transformation;
        result_pose.block<3,1>(0,3) = t_w_curr.cast<float>();

        pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
        return true;
    }
} 








