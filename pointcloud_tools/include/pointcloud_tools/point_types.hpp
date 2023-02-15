#ifndef POINTCLOUD_TOOLS_POINT_TYPES_HPP
#define POINTCLOUD_TOOLS_POINT_TYPES_HPP

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#define ADD_COVARIANCE   \
    ADD_COVARIANCE_UNION \
    ADD_EIGEN_MAPS_COVARIANCE

#define ADD_COVARIANCE_UNION     \
    union EIGEN_ALIGN16 {        \
        float covariance[6];     \
        struct {                 \
            float covariance_xx; \
            float covariance_xy; \
            float covariance_xz; \
            float covariance_yy; \
            float covariance_yz; \
            float covariance_zz; \
        };                       \
    };

#define ADD_EIGEN_MAPS_COVARIANCE                                                                               \
    inline Eigen::Matrix3f getCovarianceMatrix() {                                                              \
        Eigen::Matrix3f covariance;                                                                             \
        covariance << covariance_xx, covariance_xy, covariance_xz, covariance_xy, covariance_yy, covariance_yz, \
                covariance_xz, covariance_yz, covariance_zz;                                                    \
        return covariance;                                                                                      \
    }                                                                                                           \
    inline Eigen::Map<Eigen::Matrix<float, 6, 1>> getCovarianceVector6fMap() {                                  \
        return (Eigen::Matrix<float, 6, 1>::Map(covariance));                                                   \
    }                                                                                                           \
    inline const Eigen::Map<const Eigen::Matrix<float, 6, 1>> getCovarianceVector6fMap() const {                \
        return (Eigen::Matrix<float, 6, 1>::Map(covariance));                                                   \
    }

// Follow the conventions of the point types as in pcl/point_types.hpp.
struct EIGEN_ALIGN16 _PointCovariance {
    PCL_ADD_POINT4D;  // float: x, y, z
    ADD_COVARIANCE;   // float: covariance[6]
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointCovariance : public _PointCovariance {
    PointCovariance(const _PointCovariance& p);

    PointCovariance();

    friend std::ostream& operator<<(std::ostream& os, const PointCovariance& p);
};

std::ostream& operator<<(std::ostream& os, const PointCovariance& p);

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCovariance,
        (float, x, x)(float, y, y)(float, z, z)(float, covariance_xx, covariance_xx)(float, covariance_xy,
                covariance_xy)(float, covariance_xz, covariance_xz)(float, covariance_yy, covariance_yy)(float,
                covariance_yz, covariance_yz)(float, covariance_zz, covariance_zz))

// Follow the conventions of the point types as in pcl/point_types.hpp except don't pad 4 for curvature.
struct EIGEN_ALIGN16 _PointNormalCovariance {
    PCL_ADD_POINT4D;   // float: x, y, z
    PCL_ADD_NORMAL4D;  // float: normal_x, normal_y, normal_z
    float curvature;
    ADD_COVARIANCE;  // float: covariance[6]
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointNormalCovariance : public _PointNormalCovariance {
    PointNormalCovariance(const _PointNormalCovariance& p);

    PointNormalCovariance();

    friend std::ostream& operator<<(std::ostream& os, const PointNormalCovariance& p);
};

std::ostream& operator<<(std::ostream& os, const PointNormalCovariance& p);

POINT_CLOUD_REGISTER_POINT_STRUCT(PointNormalCovariance,
        (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z,
                normal_z)(float, curvature, curvature)(float, covariance_xx, covariance_xx)(float, covariance_xy,
                covariance_xy)(float, covariance_xz, covariance_xz)(float, covariance_yy, covariance_yy)(float,
                covariance_yz, covariance_yz)(float, covariance_zz, covariance_zz))

#endif
