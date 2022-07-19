#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
// #include "lidar_localization/subscriber/imu_subscriber.hpp"
// #include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/front_end/front_end.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<CloudPublisher> local_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
    std::shared_ptr<CloudPublisher> global_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);

    // std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();
    std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>(nh);

    std::deque<CloudData> cloud_data_buff;
    bool front_end_pose_inited = false;

    CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR current_scan_ptr(new CloudData::CLOUD());
    
    double run_time = 0.0;
    bool has_global_map_published = false;
    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();

    ros::Rate rate(100);
    while (ros::ok()) 
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);//将点云数据存放到cloud_data_buff队列中
        while (cloud_data_buff.size() > 0)
        {
            CloudData cloud_data = cloud_data_buff.front();//从队列中返回地一个元素

            cloud_data_buff.pop_front(); //将队列中的地一个元素弹出

            if (!front_end_pose_inited) 
            {
                front_end_pose_inited = true;
                front_end_ptr->SetInitPose(odometry_matrix);
            }
            front_end_ptr->SetPredictPose(odometry_matrix);
            Eigen::Matrix4f laser_matrix = front_end_ptr->Update(cloud_data);
            laser_odom_pub_ptr->Publish(laser_matrix);

            front_end_ptr->GetCurrentScan(current_scan_ptr);
            cloud_pub_ptr->Publish(current_scan_ptr);
            if (front_end_ptr->GetNewLocalMap(local_map_ptr))
                local_map_pub_ptr->Publish(local_map_ptr);

            if (run_time > 460.0 && !has_global_map_published) 
            {
                if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) 
                {
                    global_map_pub_ptr->Publish(global_map_ptr);
                    has_global_map_published = true;
                }
            }
        }

        rate.sleep();
    }


    return 0;
}