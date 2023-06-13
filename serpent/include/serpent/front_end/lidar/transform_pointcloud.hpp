#ifndef SERPENT_POINTCLOUD_TRANSFORM_HPP
#define SERPENT_POINTCLOUD_TRANSFORM_HPP

#include <pointcloud_tools/point_types.hpp>

namespace serpent {

template<typename PointT, typename Scalar>
void transform_pointcloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
        const Eigen::Matrix<Scalar, 4, 4>& transform);

template<typename PointT, typename Scalar, int Mode>
void transform_pointcloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
        const Eigen::Transform<Scalar, 3, Mode>& transform);

template<typename Scalar>
void transform_pointcloud(const pcl::PointCloud<PointNormalUnit>& cloud_in, pcl::PointCloud<PointNormalUnit>& cloud_out,
        const Eigen::Matrix<Scalar, 4, 4>& transform);

template<typename Scalar, int Mode>
void transform_pointcloud(const pcl::PointCloud<PointNormalUnit>& cloud_in, pcl::PointCloud<PointNormalUnit>& cloud_out,
        const Eigen::Transform<Scalar, 3, Mode>& transform);

template<typename Scalar>
void transform_pointcloud(const pcl::PointCloud<PointNormalCovariance>& cloud_in,
        pcl::PointCloud<PointNormalCovariance>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform);

template<typename Scalar, int Mode>
void transform_pointcloud(const pcl::PointCloud<PointNormalCovariance>& cloud_in,
        pcl::PointCloud<PointNormalCovariance>& cloud_out, const Eigen::Transform<Scalar, 3, Mode>& transform);

}

#include "serpent/front_end/lidar/transform_pointcloud-inl.hpp"

#endif
