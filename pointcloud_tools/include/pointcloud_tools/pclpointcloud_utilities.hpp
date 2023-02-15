#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_UTILITIES_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <string>

namespace pct {

template<typename PointT>
void convert_ns_to_s(pcl::PointCloud<PointT>& pointcloud) {
    for (auto& point : pointcloud) {
        point.t /= 1.0e9;
    }
}

template<typename PointIn>
int check_normals(const pcl::PointCloud<PointIn>& src, const decltype(PointIn::normal_x) threshold = 1.0e-6) {
    static_assert(std::is_floating_point<decltype(PointIn::normal_x)>::value, "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointIn::normal_y)>::value, "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointIn::normal_z)>::value, "normal_z is not a floating point type");
    int unnormalised_normals{0};
    for (std::size_t i = 0; i < src.size(); ++i) {
        const Eigen::Matrix<decltype(PointIn::normal_x), 3, 1> normal{src[i].normal_x, src[i].normal_y,
                src[i].normal_z};
        if (std::abs(normal.norm() - static_cast<decltype(PointIn::normal_x)>(1.0)) > threshold) {
            ++unnormalised_normals;
        }
    }
    return unnormalised_normals;
}

}

#endif
