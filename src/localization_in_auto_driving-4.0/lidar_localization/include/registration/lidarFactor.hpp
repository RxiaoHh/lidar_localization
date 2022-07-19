#ifndef _LIDAR_LOCALIZATION_LIDAR_FACTOR_HPP_
#define _LIDAR_LOCALIZATION_LIDAR_FACTOR_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


namespace lidar_localization
{
    struct ICP_Error
    {
        ICP_Error(Eigen::Vector3d curr_point,Eigen::Vector3d last_point)
        :curr_point_(curr_point),last_point_(last_point)
        {
        }

        template <typename T>
        bool operator()(const T *q,const T *t,T *rediduals) const
        {
            Eigen::Matrix<T,3,1> cp((T)curr_point_[0],(T)curr_point_[1],(T)curr_point_[2]); //两个点
            Eigen::Matrix<T,3,1> lp((T)last_point_[0],(T)last_point_[1],(T)last_point_[2]);

            Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
            Eigen::Matrix<T,3,1> t_last_curr(t[0],t[1],t[2]);

            Eigen::Matrix<T,3,1> lp_t;
            lp_t = q_last_curr * cp + t_last_curr;

            rediduals[0] = lp[0] - lp_t[0];
            rediduals[1] = lp[1] - lp_t[1];
            rediduals[2] = lp[2] - lp_t[2];

            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d curr_point,const Eigen::Vector3d last_point)
        {
            return (new ceres::AutoDiffCostFunction<ICP_Error,3,4,3>(new ICP_Error(curr_point,last_point)));
        }

        Eigen::Vector3d curr_point_,last_point_;
    };

}

#endif

