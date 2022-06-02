#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace pct {

template<typename PointT>
void convert_ns_to_s(pcl::PointCloud<PointT>& pointcloud) {
    for (auto& point : pointcloud) {
        point.t /= 1.0e9;
    }
}

template<typename PointIn, typename PointOut>
void copy_xyz(const pcl::PointCloud<PointIn>& src, pcl::PointCloud<PointOut>& dest) {
    if (src.size() != dest.size()) {
        throw std::runtime_error("Source and destination pointclouds do not have the same size.");
    }
    for (std::size_t i = 0; i < src.size(); ++i) {
        dest[i].x = src[i].x;
        dest[i].y = src[i].y;
        dest[i].z = src[i].z;
    }
}

}

#endif
