#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD_TRANSFORM_HPP

#include <pcl/point_cloud.h>

#include <Eigen/Geometry>
#include <eigen_ext/geometry.hpp>

#include "pointcloud_tools/point_types.hpp"

namespace pct {

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

/* Implementation */

template<typename PointT>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix4f& transform, const bool copy_all_fields) {
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

    // Code mostly copied from PCL library v1.10.
    if (&cloud_in != &cloud_out) {
        // Note: could be replaced by cloud_out = cloud_in
        cloud_out.header = cloud_in.header;
        cloud_out.width = cloud_in.width;
        cloud_out.height = cloud_in.height;
        cloud_out.is_dense = cloud_in.is_dense;
        cloud_out.points.reserve(cloud_out.points.size());
        if (copy_all_fields)
            cloud_out.points.assign(cloud_in.points.begin(), cloud_in.points.end());
        else
            cloud_out.points.resize(cloud_in.points.size());
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
        cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
    }

    pcl::detail::Transformer<float> tf(transform);
    // If the data is dense, we don't need to check for NaN
    if (cloud_in.is_dense) {
        for (std::size_t i = 0; i < cloud_out.points.size(); ++i) {
            tf.se3(cloud_in[i].data, cloud_out[i].data);
            tf.so3(cloud_in[i].data_n, cloud_out[i].data_n);
            const Eigen::Matrix3f rotation = transform.template block<3, 3>(0, 0);
            cloud_out[i].setCovariance(eigen_ext::rotate_point_covariance(cloud_in[i].getCovarianceMatrix(), rotation));
        }
    }
    // Dataset might contain NaNs and Infs, so check for them first.
    else {
        for (std::size_t i = 0; i < cloud_out.points.size(); ++i) {
            if (!std::isfinite(cloud_in.points[i].x) || !std::isfinite(cloud_in.points[i].y) ||
                    !std::isfinite(cloud_in.points[i].z))
                continue;
            tf.se3(cloud_in[i].data, cloud_out[i].data);
            tf.so3(cloud_in[i].data_n, cloud_out[i].data_n);
            const Eigen::Matrix3f rotation = transform.template block<3, 3>(0, 0);
            cloud_out[i].setCovariance(eigen_ext::rotate_point_covariance(cloud_in[i].getCovarianceMatrix(), rotation));
        }
    }
}

template<typename PointT, typename Scalar>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform, const bool copy_all_fields) {
    static_assert(std::is_floating_point<Scalar>::value, "Scalar is not a floating point type");
    transform_pointcloud_with_normals_and_covariances(cloud_in, cloud_out, transform.template cast<float>(),
            copy_all_fields);
}

template<typename PointT, typename Scalar, int Mode>
void transform_pointcloud_with_normals_and_covariances(const pcl::PointCloud<PointT>& cloud_in,
        pcl::PointCloud<PointT>& cloud_out, const Eigen::Transform<Scalar, 3, Mode>& transform,
        const bool copy_all_fields) {
    transform_pointcloud_with_normals_and_covariances(cloud_in, cloud_out, transform.matrix(), copy_all_fields);
}

}

#endif
