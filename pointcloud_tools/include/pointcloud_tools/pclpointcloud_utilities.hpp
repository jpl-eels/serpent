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

/* Implementation */

template<typename PointT>
void convert_ns_to_s(pcl::PointCloud<PointT>& pointcloud) {
    for (auto& point : pointcloud) {
        point.t /= 1.0e9;
    }
}

template<typename PointT>
int check_normals(const pcl::PointCloud<PointT>& pointcloud, const decltype(PointT::normal_x) threshold) {
    static_assert(std::is_floating_point<decltype(PointT::normal_x)>::value, "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::normal_y)>::value, "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::normal_z)>::value, "normal_z is not a floating point type");
    int unnormalised_normals{0};
    for (std::size_t i = 0; i < pointcloud.size(); ++i) {
        const Eigen::Matrix<decltype(PointT::normal_x), 3, 1> normal{pointcloud[i].normal_x, pointcloud[i].normal_y,
                pointcloud[i].normal_z};
        if (std::abs(normal.norm() - static_cast<decltype(PointT::normal_x)>(1.0)) > threshold) {
            ++unnormalised_normals;
        }
    }
    return unnormalised_normals;
}

template<typename PointT>
Eigen::Matrix<float, 3, Eigen::Dynamic> extract_unit_vectors(const pcl::PointCloud<PointT>& pointcloud) {
    static_assert(std::is_floating_point<decltype(PointT::ux)>::value, "ux is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::uy)>::value, "uy is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::uz)>::value, "uz is not a floating point type");
    Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors_(3, pointcloud.size());
    for (std::size_t i = 0; i < pointcloud.size(); ++i) {
        unit_vectors_.col(i) = Eigen::Vector3f{pointcloud[i].ux, pointcloud[i].uy, pointcloud[i].uz};
    }
    return unit_vectors_;
}

}

#endif
