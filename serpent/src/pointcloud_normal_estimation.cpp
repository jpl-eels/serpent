#include "serpent/pointcloud_normal_estimation.hpp"

#include <pcl_ros/point_cloud.h>

#include <pointcloud_tools/pclpointcloud_utilities.hpp>

namespace serpent {

PointcloudNormalEstimation::PointcloudNormalEstimation()
    : nh("~") {
    // Publisher
    normal_pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>("normal_estimation/pointcloud", 1);

    // Subscriber
    normal_estimation_pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>("filter/filtered_pointcloud", 100,
            &PointcloudNormalEstimation::normal_estimation_callback, this);

    // Normal Estimation Configuration
    normal_estimation_tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    normal_estimation.setSearchMethod(normal_estimation_tree);
    normal_estimation.setNumberOfThreads(nh.param<int>("normal_estimation/threads", 1));
    normal_estimation.setViewPoint(nh.param<float>("normal_estimation/viewpoint/x", 0.f),
            nh.param<float>("normal_estimation/viewpoint/y", 0.f),
            nh.param<float>("normal_estimation/viewpoint/z", std::numeric_limits<float>::max()));
    const std::string ne_method = nh.param<std::string>("normal_estimation/method", "knn");
    if (ne_method == "knn") {
        normal_estimation.setKSearch(nh.param<int>("normal_estimation/k", 30));
    } else if (ne_method == "radius") {
        normal_estimation.setRadiusSearch(nh.param<double>("normal_estimation/radius", 1.0));
    } else {
        throw std::runtime_error("Unknown normal estimation method " + ne_method);
    }
}

void PointcloudNormalEstimation::normal_estimation_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    // Extract fields for normal estimation
    auto pointcloud_in = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*msg, *pointcloud_in);

    // Compute normals
    auto pointcloud = compute(pointcloud_in, normal_estimation);

    // Copy other fields
    pct::copy_xyz(*pointcloud_in, *pointcloud);

    // Publish pointcloud
    normal_pointcloud_publisher.publish(pointcloud);
}

}
