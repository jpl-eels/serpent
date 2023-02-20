#ifndef POINTCLOUD_TOOLS_IMPL_COMPUTE_DISTANCE_HPP
#define POINTCLOUD_TOOLS_IMPL_COMPUTE_DISTANCE_HPP

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <eigen_ros/eigen_ros.hpp>

#include "pointcloud_tools/compute_distances.hpp"

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
    auto output = boost::make_shared<pcl::PCLPointCloud2>();
    output->header = source.header;
    output->height = 1;
    output->width = source.size();
    const auto x_type = pcl::PCLPointField::PointFieldTypes::FLOAT32;
    const auto y_type = pcl::PCLPointField::PointFieldTypes::FLOAT32;
    const auto z_type = pcl::PCLPointField::PointFieldTypes::FLOAT32;
    const auto distance_type = pcl::PCLPointField::PointFieldTypes::FLOAT32;
    output->fields.push_back(pcl::PCLPointField{.name = "x", .offset = 0, .datatype = x_type, .count = 1});
    output->fields.push_back(pcl::PCLPointField{.name = "y",
            .offset =
                    output->fields.back().offset +
                    output->fields.back().count * static_cast<std::uint32_t>(sizeof(pcl::traits::asType<x_type>::type)),
            .datatype = y_type,
            .count = 1});
    output->fields.push_back(pcl::PCLPointField{.name = "z",
            .offset =
                    output->fields.back().offset +
                    output->fields.back().count * static_cast<std::uint32_t>(sizeof(pcl::traits::asType<y_type>::type)),
            .datatype = z_type,
            .count = 1});
    output->fields.push_back(pcl::PCLPointField{.name = "distance",
            .offset =
                    output->fields.back().offset +
                    output->fields.back().count * static_cast<std::uint32_t>(sizeof(pcl::traits::asType<z_type>::type)),
            .datatype = distance_type,
            .count = 1});
    output->point_step =
            output->fields.back().offset +
            output->fields.back().count * static_cast<std::uint32_t>(sizeof(pcl::traits::asType<distance_type>::type));
    output->row_step = output->point_step * output->width;
    output->data.resize(output->row_step * output->height);
    output->is_dense = false;

    // Compute nearest neighbours
    typename pcl::KdTreeFLANN<PointTarget> kdtree;
    kdtree.setInputCloud(transformed_target);
    const int k{1};
    std::vector<int> knn_indices(k);
    std::vector<float> knn_square_distances(k);
    for (std::size_t i = 0; i < source.size(); ++i) {
        *reinterpret_cast<float*>(&output->data.at(output->point_step * i + output->fields[0].offset)) =
                static_cast<float>(source[i].x);
        *reinterpret_cast<float*>(&output->data.at(output->point_step * i + output->fields[1].offset)) =
                static_cast<float>(source[i].y);
        *reinterpret_cast<float*>(&output->data.at(output->point_step * i + output->fields[2].offset)) =
                static_cast<float>(source[i].z);
        if (kdtree.nearestKSearch(source[i], k, knn_indices, knn_square_distances) == k) {
            *reinterpret_cast<float*>(&output->data.at(output->point_step * i + output->fields[3].offset)) =
                    std::sqrt(knn_square_distances[0]);
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
