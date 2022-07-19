#include "/home/renxiao/rx_ws/src/localization_in_auto_driving-4.0/lidar_localization/include/registration/icp_svd_registration.hpp"

namespace lidar_localization
{
    ICP_SVD_Registration::ICP_SVD_Registration()
    :kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()),
     cloud_tmp_(new CloudData::CLOUD()),
     input_source_(new CloudData::CLOUD())
    {
        max_iter_ = 35;
    }
    ICP_SVD_Registration::~ICP_SVD_Registration()
    {
    }

    bool ICP_SVD_Registration::SetInputTraget(CloudData::CLOUD_PTR& cloud_traget)
    {
        cloud_tmp_ = cloud_traget;
        kdtree_->setInputCloud(cloud_traget);
        return true;
    }

    size_t ICP_SVD_Registration::getCorrespondence(const CloudData::CLOUD_PTR &input_source,
                                                   std::vector<Eigen::Vector3f> &xs,
                                                   std::vector<Eigen::Vector3f> &ys)
    {
        int k = 1;
        size_t number_point = 0;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquareDistance(k);

        for(int i = 0;i < input_source->points.size();i++)
        {
            kdtree_->nearestKSearch(input_source->at(i),k,pointIdxNKNSearch,pointNKNSquareDistance);//寻找最近点
            if(pointNKNSquareDistance.at(0) > 1.0)
                continue;

            Eigen::Vector3f xs_(cloud_tmp_->at(pointIdxNKNSearch[0]).x,cloud_tmp_->at(pointIdxNKNSearch[0]).y,cloud_tmp_->at(pointIdxNKNSearch[0]).z);//目标点云
            Eigen::Vector3f ys_(input_source->at(i).x,input_source->at(i).y,input_source->at(i).z);//原始点云
            xs.push_back(xs_);
            ys.push_back(ys_);
            ++number_point;
        }
        return number_point;
    }

    void ICP_SVD_Registration::getTransform(const std::vector<Eigen::Vector3f> &xs,//目标点云
                                            const std::vector<Eigen::Vector3f> &ys,//原始点云
                                            Eigen::Matrix4f &transformation)
    {
        //计算均值
        Eigen::Vector3f sum_xs,sum_ys;
        Eigen::Vector3f mean_xs,mean_ys;

        const int N = xs.size();
        const int number_points = xs.size();
        Eigen::MatrixXf A(3,number_points);//目标点云
        Eigen::MatrixXf B(3,number_points);//原始点云

        for(int i = 0;i < number_points;i++)
        {
            A.block<3,1>(0,i) = xs[i];
            B.block<3,1>(0,i) = ys[i];
        }
        //求均值
        Eigen::Vector3f mean_x = A.rowwise().mean(); 
        Eigen::Vector3f mean_y = B.rowwise().mean(); 
        //去质心
        A.colwise() -= mean_x;
        B.colwise() -= mean_y;
        //求H矩阵
        Eigen::Matrix3f H = B * A.transpose();
        //SVD分解
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(H,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector3f t = mean_x - R * mean_y;
        
        transformation.block<3,3>(0,0) = R;
        transformation.block<3,1>(0,3) = t; 
    }

    bool ICP_SVD_Registration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                         const Eigen::Matrix4f& predict_pose, 
                                         CloudData::CLOUD_PTR& result_cloud_ptr,
                                         Eigen::Matrix4f& result_pose)
    {        
        // input_source_ = input_source;

        CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
        pcl::transformPointCloud(*input_source, *transformed_input_source, predict_pose);

        int curr_iter = 0;
        while(curr_iter < max_iter_) 
        {
            CloudData::CLOUD_PTR curr_input_source(new CloudData::CLOUD());
            pcl::transformPointCloud(*transformed_input_source,*curr_input_source,transform_matrix_);
            //获取对应点
            std::vector<Eigen::Vector3f> xs;
            std::vector<Eigen::Vector3f> ys;
            if(getCorrespondence(curr_input_source,xs,ys) < 3) 
            {
                break;
            }
            //根据svd求解出的变换矩阵，更新变换矩阵
            Eigen::Matrix4f delta_transformation;
            getTransform(xs,ys,delta_transformation);

            transform_matrix_ = delta_transformation * transform_matrix_;
            ++curr_iter;
        }
        result_pose = transform_matrix_ * predict_pose;
        Eigen::Quaternionf qr(result_pose.block<3,3>(0,0));
        qr.normalize();//归一化
        Eigen::Vector3f t = result_pose.block<3,1>(0,3);
        result_pose.block<3,3>(0,0) = qr.toRotationMatrix();
        result_pose.block<3,1>(0,3) = t;
        pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

        return true;

    }                   

}
