#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP

#include <pcl/point_cloud.h>

#include <Eigen/Geometry>

namespace pct {

template<typename PointT>
void transform_pointcloud_with_normals_and_unit_vectors(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix4f& transform, const bool copy_all_fields = true);

template<typename PointT, typename Scalar>
void transform_pointcloud_with_normals_and_unit_vectors(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform,
        const bool copy_all_fields = true);

template<typename PointT, typename Scalar, int Mode>
void transform_pointcloud_with_normals_and_unit_vectors(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Transform<Scalar, 3, Mode>& transform,
        const bool copy_all_fields = true);

template<typename PointT>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix4f& transform, const bool copy_all_fields = true);

template<typename PointT, typename Scalar>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform,
        const bool copy_all_fields = true);

template<typename PointT, typename Scalar, int Mode>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Transform<Scalar, 3, Mode>& transform,
        const bool copy_all_fields = true);

}

#include "pointcloud_tools/impl/pclpointcloud_transform.hpp"

#endif
