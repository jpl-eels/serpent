#ifndef POINTCLOUD_TOOLS_POINT_TYPES_HPP
#define POINTCLOUD_TOOLS_POINT_TYPES_HPP

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define ADD_UNIT_VECTOR   \
    ADD_UNIT_VECTOR_UNION \
    ADD_EIGEN_MAPS_UNIT_VECTOR

#define ADD_UNIT_VECTOR_UNION \
    union EIGEN_ALIGN16 {     \
        float data_u[4];      \
        float u[3];           \
        struct {              \
            float ux;         \
            float uy;         \
            float uz;         \
        };                    \
    };

#define ADD_EIGEN_MAPS_UNIT_VECTOR                                                                      \
    inline pcl::Vector3fMap getUnitVector3fMap() { return (pcl::Vector3fMap(data_u)); }                 \
    inline pcl::Vector3fMapConst getUnitVector3fMap() const { return (pcl::Vector3fMapConst(data_u)); } \
    inline pcl::Vector4fMap getUnitVector4fMap() { return (pcl::Vector4fMap(data_u)); }                 \
    inline pcl::Vector4fMapConst getUnitVector4fMap() const { return (pcl::Vector4fMapConst(data_u)); } \
    inline Eigen::Matrix3f get_unit_outer_product() const {                                             \
        return getUnitVector3fMap() * getUnitVector3fMap().transpose();                                 \
    }

#define ADD_COVARIANCE   \
    ADD_COVARIANCE_UNION \
    ADD_EIGEN_FUNCTIONS_COVARIANCE

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

#define ADD_EIGEN_FUNCTIONS_COVARIANCE                                                                          \
    inline Eigen::Matrix3f getCovarianceMatrix() const {                                                        \
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
    }                                                                                                           \
    inline void setCovariance(const Eigen::Matrix3f& matrix) {                                                  \
        covariance_xx = matrix(0, 0);                                                                           \
        covariance_xy = matrix(0, 1);                                                                           \
        covariance_xz = matrix(0, 2);                                                                           \
        covariance_yy = matrix(1, 1);                                                                           \
        covariance_yz = matrix(1, 2);                                                                           \
        covariance_zz = matrix(2, 2);                                                                           \
    }

#define ADD_ANGLEAXIS   \
    ADD_ANGLEAXIS_UNION \
    ADD_EIGEN_FUNCTIONS_ANGLEAXIS

#define ADD_ANGLEAXIS_UNION \
    union EIGEN_ALIGN16 {   \
        float data_r[4];    \
        float r[3];         \
        struct {            \
            float rx;       \
            float ry;       \
            float rz;       \
        };                  \
    };

#define ADD_EIGEN_FUNCTIONS_ANGLEAXIS                                                                               \
    inline float angle() const { return std::sqrt(rx * rx + ry * ry + rz * rz); }                                   \
    inline Eigen::AngleAxisf getAngleAxis() const {                                                                 \
        return Eigen::AngleAxisf{angle(), Eigen::Vector3f{rx, ry, rz}.normalized()};                                \
    }                                                                                                               \
    inline Eigen::Matrix3f getRotationMatrix() { return getAngleAxis().toRotationMatrix(); }                        \
    inline Eigen::Quaternionf getQuaternion() { return Eigen::Quaternionf{getAngleAxis()}; }                        \
    inline Eigen::Isometry3f getIsometry() {                                                                        \
        return Eigen::Translation<float, 3>{Eigen::Vector3f{x, y, z}} * getAngleAxis();                             \
    }                                                                                                               \
    inline void setAngleAxis(const Eigen::AngleAxisf& angle_axis) {                                                 \
        Eigen::Vector3f angle_axis_vector = angle_axis.axis() * angle_axis.angle();                                 \
        rx = angle_axis_vector[0];                                                                                  \
        ry = angle_axis_vector[1];                                                                                  \
        rz = angle_axis_vector[2];                                                                                  \
    }                                                                                                               \
    inline void setAngleAxis(const Eigen::Quaternionf& quaternion) { setAngleAxis(Eigen::AngleAxisf{quaternion}); } \
    inline void setAngleAxis(const Eigen::Matrix3f& rotation_matrix) {                                              \
        setAngleAxis(Eigen::AngleAxisf{rotation_matrix});                                                           \
    }

// Follow the conventions of the point types as in pcl/point_types.hpp.
struct EIGEN_ALIGN16 _PointUnit {
    PCL_ADD_POINT4D;  // float: x, y, z
    ADD_UNIT_VECTOR;  // float: ux, uy, uz
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointUnit : public _PointUnit {
    PointUnit(const _PointUnit& p);

    PointUnit();

    friend std::ostream& operator<<(std::ostream& os, const PointUnit& p);
};

std::ostream& operator<<(std::ostream& os, const PointUnit& p);

POINT_CLOUD_REGISTER_POINT_STRUCT(PointUnit,
        (float, x, x)(float, y, y)(float, z, z)(float, ux, ux)(float, uy, uy)(float, uz, uz))

// Follow the conventions of the point types as in pcl/point_types.hpp except don't pad 4 for curvature.
struct EIGEN_ALIGN16 _PointNormalUnit {
    PCL_ADD_POINT4D;   // float: x, y, z
    PCL_ADD_NORMAL4D;  // float: normal_x, normal_y, normal_z
    float curvature;   // float: curvature
    ADD_UNIT_VECTOR;   // float: ux, uy, uz
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointNormalUnit : public _PointNormalUnit {
    PointNormalUnit(const _PointNormalUnit& p);

    PointNormalUnit();

    friend std::ostream& operator<<(std::ostream& os, const PointNormalUnit& p);
};

std::ostream& operator<<(std::ostream& os, const PointNormalUnit& p);

POINT_CLOUD_REGISTER_POINT_STRUCT(PointNormalUnit,
        (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z,
                normal_z)(float, curvature, curvature)(float, ux, ux)(float, uy, uy)(float, uz, uz))

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
    float curvature;   // float: curvature
    ADD_COVARIANCE;    // float: covariance[6]
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

struct EIGEN_ALIGN16 _PointAngleAxis {
    PCL_ADD_POINT4D;  // float: x, y, z
    ADD_ANGLEAXIS;    // float: rx, ry, rz
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointAngleAxis : public _PointAngleAxis {
    PointAngleAxis(const _PointAngleAxis& p);

    PointAngleAxis(const Eigen::Isometry3f& pose);

    PointAngleAxis(const Eigen::Vector3f& position, const Eigen::AngleAxisf& angle_axis);

    PointAngleAxis(const Eigen::Vector3f& position, const Eigen::Quaternionf& quaternion);

    PointAngleAxis(const Eigen::Translation<float, 3>& position, const Eigen::Quaternionf& quaternion);

    PointAngleAxis(const Eigen::Translation<float, 3>& position, const Eigen::Matrix3f& rotation_matrix);

    PointAngleAxis();

    inline void setPose(const Eigen::Isometry3f& pose) {
        setPose(pose.translation(), pose.rotation());
    }

    inline void setPose(const Eigen::Vector3f& position, const Eigen::AngleAxisf& angle_axis) {
        getVector3fMap() = Eigen::Vector3f{position.x(), position.y(), position.z()};
        setAngleAxis(angle_axis);
    }

    inline void setPose(const Eigen::Vector3f& position, const Eigen::Quaternionf& quaternion) {
        setPose(position, Eigen::AngleAxisf{quaternion});
    }

    inline void setPose(const Eigen::Vector3f& position, const Eigen::Matrix3f& rotation_matrix) {
        setPose(position, Eigen::AngleAxisf{rotation_matrix});
    }

    inline void setPose(const Eigen::Translation<float, 3>& position, const Eigen::Quaternionf& quaternion) {
        setPose(Eigen::Vector3f{position.x(), position.y(), position.z()}, quaternion);
    }

    inline void setPose(const Eigen::Translation<float, 3>& position, const Eigen::Matrix3f& rotation_matrix) {
        setPose(Eigen::Vector3f{position.x(), position.y(), position.z()}, rotation_matrix);
    }

    friend std::ostream& operator<<(std::ostream& os, const PointAngleAxis& p);
};

std::ostream& operator<<(std::ostream& os, const PointAngleAxis& p);

POINT_CLOUD_REGISTER_POINT_STRUCT(PointAngleAxis,
        (float, x, x)(float, y, y)(float, z, z)(float, rx, rx)(float, ry, ry)(float, rz, rz))

#endif
