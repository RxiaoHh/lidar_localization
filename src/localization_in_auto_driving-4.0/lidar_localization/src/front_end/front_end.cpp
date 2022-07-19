#include "lidar_localization/front_end/front_end.hpp"

#include <cmath>
#include <pclomp/ndt_omp.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

namespace lidar_localization 
{
    FrontEnd::FrontEnd(ros::NodeHandle nh)
        :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
        ndt_omp_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
        local_map_ptr_(new CloudData::CLOUD()),
        global_map_ptr_(new CloudData::CLOUD()),
        result_cloud_ptr_(new CloudData::CLOUD()),
        nh_(nh)
    {
        std::string fin = "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/config/config.yaml";
        YAML::Node yamlConfig = YAML::LoadFile(fin);
        std::string registration_method = yamlConfig["registration_method"].as<std::string>();

        cloud_set_leaf_size_ = 1.3;
        local_set_leaf_size_ = 0.6;
        dis_set_leaf_size_ = 0.5;

        // cloud_set_leaf_size_ = yamlConfig["cloud_set_leaf_size"].as<double>();
        // local_set_leaf_size_ = yamlConfig["local_set_leaf_size"].as<double>();
        // dis_set_leaf_size_ = yamlConfig["dis_set_leaf_size"].as<double>();

        // 给个默认参数，以免类的使用者在匹配之前忘了设置参数
        cloud_filter_.setLeafSize(cloud_set_leaf_size_,cloud_set_leaf_size_,cloud_set_leaf_size_);
        local_map_filter_.setLeafSize(local_set_leaf_size_,local_set_leaf_size_,local_set_leaf_size_);
        display_filter_.setLeafSize(dis_set_leaf_size_,dis_set_leaf_size_,dis_set_leaf_size_);
  
        if(registration_method == "NDT")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            registration_ptr_ = std::make_shared<NDT_Registration>();
        }
        else if(registration_method == "NDT_OMP")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            registration_ptr_ = std::make_shared<NDT_OMP_Registation>();   
        }
        else if(registration_method == "ICP")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            registration_ptr_ = std::make_shared<ICP_Registration>(); 
        }
        else if(registration_method == "SVD_ICP")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            registration_ptr_ = std::make_shared<ICP_SVD_Registration>();             
        }
        else if(registration_method == "ICP_GN")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            registration_ptr_ = std::make_shared<ICP_GN_Registration>();
        }
        else if(registration_method == "ICP_CERES")
        {
            std::cout << "registration method is " << registration_method << std::endl;
            // registration_ptr_ = std::make_shared<opt_ICP_CERES>();
            registration_ptr_ = std::make_shared<ICP_CERES_Registrtion>();
        }

    }

    Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) 
    {
        current_frame_.cloud_data.time = cloud_data.time;
        std::vector<int> indices;
        //移除无效点
        pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
        //滤波
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
        cloud_filter_.filter(*filtered_cloud_ptr);

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;

        // 局部地图容器中没有关键帧，代表是第一帧数据
        // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
        if (local_map_frames_.size() == 0) 
        {
            current_frame_.pose = init_pose_;
            UpdateNewFrame(current_frame_);
            return current_frame_.pose;
        }

        // 不是第一帧，就正常匹配
        ros::Time t1 = ros::Time::now();
        registration_ptr_->ScanMatch(filtered_cloud_ptr,predict_pose,result_cloud_ptr_,current_frame_.pose);
        ros::Time t2 = ros::Time::now();
        // std::cout << "registrate time is " << (t2 - t1).toSec() << " s" << std::endl;

        // 更新相邻两帧的相对运动
        step_pose = last_pose.inverse() * current_frame_.pose;
        predict_pose = current_frame_.pose * step_pose; //作用在于作为ndt匹配的初值
        last_pose = current_frame_.pose;

        // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
        if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
            fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
            fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2.0) 
        {
            UpdateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

        return current_frame_.pose;
    }


    bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) 
    {
        init_pose_ = init_pose;
        return true;
    }

    bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) 
    {
        predict_pose_ = predict_pose;
        return true;
    }

    void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) 
    {
        Frame key_frame = new_key_frame;
        // 这一步的目的是为了把关键帧的点云保存下来
        // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
        // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
        key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
        
        // 更新局部地图
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > 20) 
        {
            local_map_frames_.pop_front();
        }
        local_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < local_map_frames_.size(); ++i) 
        {
            pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                     *transformed_cloud_ptr, 
                                     local_map_frames_.at(i).pose);
            *local_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_local_map_ = true;

        // 更新ndt匹配的目标点云
        if (local_map_frames_.size() < 10) 
        {
            registration_ptr_->SetInputTraget(local_map_ptr_);
        } 
        else 
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_.setInputCloud(local_map_ptr_);
            local_map_filter_.filter(*filtered_local_map_ptr);

            registration_ptr_->SetInputTraget(filtered_local_map_ptr);
        }

        // 更新全局地图
        global_map_frames_.push_back(key_frame);
        if (global_map_frames_.size() % 100 != 0) 
        {
            return;
        } 
        else 
        {
            global_map_ptr_.reset(new CloudData::CLOUD());
            for (size_t i = 0; i < global_map_frames_.size(); ++i) 
            {
                pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, 
                                         *transformed_cloud_ptr, 
                                         global_map_frames_.at(i).pose);
                *global_map_ptr_ += *transformed_cloud_ptr;
            }
            has_new_global_map_ = true;
        }
    }

    bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) 
    {
        if (has_new_local_map_) 
        {
            display_filter_.setInputCloud(local_map_ptr_);
            display_filter_.filter(*local_map_ptr);
            return true;
        }
        return false;
    }

    bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) 
    {
        if (has_new_global_map_) 
        {
            display_filter_.setInputCloud(global_map_ptr_);
            display_filter_.filter(*global_map_ptr);
            return true;
        }
        return false;
    }

    bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) 
    {
        display_filter_.setInputCloud(result_cloud_ptr_);
        display_filter_.filter(*current_scan_ptr);
        return true;
    }
}