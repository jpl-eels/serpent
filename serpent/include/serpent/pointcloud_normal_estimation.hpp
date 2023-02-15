#ifndef SERPENT_POINTCLOUD_NORMAL_ESTIMATION_HPP
#define SERPENT_POINTCLOUD_NORMAL_ESTIMATION_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace serpent {

template<typename PointIn = pcl::PointXYZ, typename PointOut = pcl::PointNormal>
class PointcloudNormalEstimation {
    static_assert(std::is_floating_point<decltype(PointIn::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointIn::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointIn::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointOut::normal_x)>::value, "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointOut::normal_y)>::value, "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointOut::normal_z)>::value, "normal_z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointOut::curvature)>::value,
            "curvature is not a floating point type");
public:
    using PointCloudIn = typename pcl::PointCloud<PointIn>;
    using PointCloudOut = typename pcl::PointCloud<PointOut>;

    explicit PointcloudNormalEstimation();

private:
    void normal_estimation_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Publisher normal_pointcloud_publisher;
    ros::Subscriber normal_estimation_pointcloud_subscriber;

    // Normal Estimation
    typename pcl::search::KdTree<PointIn>::Ptr normal_estimation_tree;
    typename pcl::NormalEstimationOMP<PointIn, PointOut> normal_estimation;
};

template<typename PointIn, typename PointOut>
typename pcl::PointCloud<PointOut>::Ptr compute(const typename pcl::PointCloud<PointIn>::ConstPtr pointcloud_in,
        typename pcl::Feature<PointIn, PointOut>& feature) {
    feature.setInputCloud(pointcloud_in);
    auto pointcloud = boost::make_shared<pcl::PointCloud<PointOut>>();
    feature.compute(*pointcloud);
    return pointcloud;
}

template<typename PointT>
void compute_in_place(const typename pcl::PointCloud<PointT>::Ptr pointcloud,
        typename pcl::Feature<PointT, PointT>& feature) {
    feature.setInputCloud(pointcloud);
    feature.compute(*pointcloud);
}

}

#include "serpent/impl/pointcloud_normal_estimation.hpp"

#endif
