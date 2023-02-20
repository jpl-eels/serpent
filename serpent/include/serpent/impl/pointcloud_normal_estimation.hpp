#ifndef SERPENT_IMPL_POINTCLOUD_NORMAL_ESTIMATION_HPP
#define SERPENT_IMPL_POINTCLOUD_NORMAL_ESTIMATION_HPP

#include <pcl_ros/point_cloud.h>

#include <pointcloud_tools/pclpointcloud_utilities.hpp>

#include "serpent/pointcloud_normal_estimation.hpp"
#include "serpent/registration_covariance.hpp"

namespace serpent {

template<typename PointIn, typename PointOut>
PointcloudNormalEstimation<PointIn, PointOut>::PointcloudNormalEstimation(const std::string& input_topic)
    : nh("serpent") {
    // Publisher
    normal_pointcloud_publisher = nh.advertise<PointCloudOut>("normal_estimation/pointcloud", 1);

    // Subscriber
    normal_estimation_pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100,
            &PointcloudNormalEstimation::normal_estimation_callback, this);

    // Normal Estimation Configuration
    normal_estimation_tree = boost::make_shared<typename pcl::search::KdTree<PointIn>>();
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

/**
 * @brief We are faced with one of two annoying choices here. When PCL converts from pcl::PCLPointCloud2 to
 * pcl::PointCloud<PointT>, it creates a mapping from every type (called a Tag) in PointT and finds a field with the
 * same name in the field list of pcl::PCLPointCloud2. Thus if we lack the normal and curvature fields in
 * pcl::PCLPointCloud2, PCL will throw an warning every time for each missing field when creating this mapping.
 *
 * One solution is to convert to pcl::PointCloud<PointT1> where PointT1 contains the fields in the pcl::PCLPointCloud2
 * that we need, and then compute the normals in a second cloud, pcl::PointCloud<PointT2> which is PointT1 with the
 * inclusion of normals. The disadvantage of this method is that we require PointT1 and PointT2 defined, which may
 * require custom types.
 *
 * Another solution is to add more fields for the normals and curvature to the field list ("augmented_msg_fields" below)
 * of the received message and then call:
 *  pcl::MsgFieldMap field_map;
 *  createMapping(const std::vector<pcl::PCLPointField>& augmented_msg_fields, MsgFieldMap& field_map);
 *  fromPCLPointCloud2(const pcl::PCLPointCloud2& in, pcl::PointCloud<PointT>& out, const MsgFieldMap& field_map)
 * The effect of this would be to copy other data into the normal and curvature fields (e.g. the x, y, z, pad data).
 *
 * Another solution would be to create a pcl::MsgFieldMap in another way that avoids the warning, however it ought to be
 * done well since pcl's current method does some optimisation for copying adjacent field data.
 *
 * @tparam PointIn
 * @tparam PointOut
 * @param msg
 */
template<typename PointIn, typename PointOut>
void PointcloudNormalEstimation<PointIn, PointOut>::normal_estimation_callback(
        const pcl::PCLPointCloud2::ConstPtr& msg) {
    // Extract fields for normal estimation
    auto pointcloud_in = boost::make_shared<PointCloudIn>();
    pcl::fromPCLPointCloud2(*msg, *pointcloud_in);

    // Compute normals
    auto pointcloud_out = compute(pointcloud_in, normal_estimation);

    // Copy other fields
    pcl::copyPointCloud(*pointcloud_in, *pointcloud_out);

    // Publish pointcloud
    normal_pointcloud_publisher.publish(pointcloud_out);
}

}

#endif
