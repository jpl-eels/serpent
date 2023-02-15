#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP

#include <pcl/point_cloud.h>

#include <Eigen/Geometry>

#include "pointcloud_tools/point_types.hpp"

// TODO: take other transform types.

namespace pct {

template<typename PointT, typename Scalar>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform);

/* Implementation */

template<typename PointT, typename Scalar>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform) {
    static_assert(std::is_floating_point<Scalar>::value, "Scalar is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::normal_x)>::value, "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::normal_y)>::value, "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::normal_z)>::value, "normal_z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_xx)>::value,
            "covariance_xx is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_xy)>::value,
            "covariance_xy is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_xz)>::value,
            "covariance_xz is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_yy)>::value,
            "covariance_yy is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_yz)>::value,
            "covariance_yz is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointT::covariance_zz)>::value,
            "covariance_zz is not a floating point type");
    throw std::runtime_error("pct::transform_pointcloud_with_normals_and_covariances not implemented.");
}

}

#endif
