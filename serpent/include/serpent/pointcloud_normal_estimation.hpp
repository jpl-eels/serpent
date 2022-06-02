#ifndef SERPENT_POINTCLOUD_NORMAL_ESTIMATION_HPP
#define SERPENT_POINTCLOUD_NORMAL_ESTIMATION_HPP

#include <pcl/features/normal_3d_omp.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace serpent {

class PointcloudNormalEstimation {
public:
    explicit PointcloudNormalEstimation();

private:
    void normal_estimation_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Publisher normal_pointcloud_publisher;
    ros::Subscriber normal_estimation_pointcloud_subscriber;

    // Normal Estimation
    pcl::search::KdTree<pcl::PointXYZ>::Ptr normal_estimation_tree;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
};

template<typename PointIn, typename PointOut>
typename pcl::PointCloud<PointOut>::Ptr compute(const typename pcl::PointCloud<PointIn>::ConstPtr& msg,
        typename pcl::Feature<PointIn, PointOut>& feature) {
    feature.setInputCloud(msg);
    auto pointcloud = boost::make_shared<pcl::PointCloud<PointOut>>();
    feature.compute(*pointcloud);
    return pointcloud;
}

}

#endif
