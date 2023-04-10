#ifndef POINTCLOUD_TOOLS_IMPL_COMPUTE_DISTANCE_HPP
#define POINTCLOUD_TOOLS_IMPL_COMPUTE_DISTANCE_HPP

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <eigen_ros/eigen_ros.hpp>

#include "pointcloud_tools/compute_distances.hpp"
#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

template<typename PointSource, typename PointTarget>
ComputeDistances<PointSource, PointTarget>::ComputeDistances()
    : nh("~"),
      tf_listener(tf_buffer),
      sync(10) {
    source_subscriber.subscribe(nh, "input_source", 10);
    target_subscriber.subscribe(nh, "input_target", 10);
    sync.connectInput(source_subscriber, target_subscriber);
    sync.registerCallback(boost::bind(&ComputeDistances::callback, this, _1, _2));
    output_publisher = nh.advertise<pcl::PCLPointCloud2>("output", 1);
}

template<typename PointSource, typename PointTarget>
void ComputeDistances<PointSource, PointTarget>::callback(const PointCloudSourceConstPtr& source,
        const PointCloudTargetConstPtr& target) {
    const geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(source->header.frame_id,
            target->header.frame_id, pcl_conversions::fromPCL(source->header.stamp), ros::Duration(0.1));
    const Eigen::Isometry3d transform = eigen_ros::from_ros<Eigen::Isometry3d>(tf.transform);
    const auto transformed_target = transform_target(*target, transform, source->header.frame_id);
    const auto distance_cloud = compute_distances(*source, transformed_target);
    output_publisher.publish(distance_cloud);
}

template<typename PointSource, typename PointTarget>
pcl::PCLPointCloud2::ConstPtr ComputePointToPointDistances<PointSource, PointTarget>::compute_distances(
        const PointCloudSource& source, const PointCloudTargetConstPtr transformed_target) const {
    // Set up pointcloud
    auto output = boost::make_shared<pcl::PCLPointCloud2>(add_fields(pcl::PCLPointCloud2(), {"x", "y", "z", "distance"},
            pcl::PCLPointField::PointFieldTypes::FLOAT32));
    output->header = source.header;
    output->height = 1;
    output->width = source.size();
    output->row_step = output->point_step * output->width;
    output->data.resize(output->row_step * output->height);
    output->is_dense = false;

    // Compute nearest neighbours
    typename pcl::KdTreeFLANN<PointTarget> kdtree;
    kdtree.setInputCloud(transformed_target);
    const int k{1};
    std::vector<int> knn_indices(k);
    std::vector<float> knn_square_distances(k);
    auto set_x_field_data = create_set_field_data_function<float, float>(get_field(*output, "x"));
    auto set_y_field_data = create_set_field_data_function<float, float>(get_field(*output, "y"));
    auto set_z_field_data = create_set_field_data_function<float, float>(get_field(*output, "z"));
    auto set_d_field_data = create_set_field_data_function<float, float>(get_field(*output, "distance"));
    const std::size_t num_points = source.size();
    for (std::size_t i = 0; i < num_points; ++i) {
        set_x_field_data(*output, i, static_cast<float>(source[i].x));
        set_y_field_data(*output, i, static_cast<float>(source[i].y));
        set_z_field_data(*output, i, static_cast<float>(source[i].z));
        if (kdtree.nearestKSearch(source[i], k, knn_indices, knn_square_distances) == k) {
            set_d_field_data(*output, i, std::sqrt(knn_square_distances[0]));
        } else {
            throw std::runtime_error("Failed to find nearest neighbour.");
        }
    }
    return output;
}

template<typename PointSource, typename PointTarget>
typename ComputePointToPointDistances<PointSource, PointTarget>::PointCloudTargetConstPtr
ComputePointToPointDistances<PointSource, PointTarget>::transform_target(const PointCloudTarget& target,
        const Eigen::Isometry3d& transform, const std::string& frame_id) const {
    auto output = boost::make_shared<PointCloudTarget>();
    pcl::transformPointCloud(target, *output, transform.cast<float>());
    output->header.frame_id = frame_id;
    return output;
}

}

#endif
