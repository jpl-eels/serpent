#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <string>

namespace pct {

template<typename PointT>
void convert_ns_to_s(pcl::PointCloud<PointT>& pointcloud);

template<typename PointT>
int check_normals(const pcl::PointCloud<PointT>& pointcloud, const decltype(PointT::normal_x) threshold = 1.0e-6);

template<typename PointT>
Eigen::Matrix<float, 3, Eigen::Dynamic> extract_unit_vectors(const pcl::PointCloud<PointT>& pointcloud);

}

#include "pointcloud_tools/impl/pclpointcloud_utilities.hpp"

#endif
