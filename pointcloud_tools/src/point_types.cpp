#include <pointcloud_tools/point_types.hpp>

PointUnit::PointUnit(const _PointUnit& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    ux = p.ux;
    uy = p.uy;
    uz = p.uz;
    data_u[3] = 0.0f;
}

PointUnit::PointUnit() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    ux = uy = uz = 0.f;
    data_u[3] = 0.0f;
}

std::ostream& operator<<(std::ostream& os, const PointUnit& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.ux << ", " << p.uy << ", " << p.uz << ")";
    return (os);
}

PointNormalUnit::PointNormalUnit(const _PointNormalUnit& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    normal_x = p.normal_x;
    normal_y = p.normal_y;
    normal_z = p.normal_z;
    data_n[3] = 0.0f;
    curvature = p.curvature;
    ux = p.ux;
    uy = p.uy;
    uz = p.uz;
    data_u[3] = 0.0f;
}

PointNormalUnit::PointNormalUnit() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    curvature = 0.f;
    ux = uy = uz = 0.f;
    data_u[3] = 0.0f;
}

std::ostream& operator<<(std::ostream& os, const PointNormalUnit& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2]
       << " - " << p.ux << ", " << p.uy << ", " << p.uz << ")";
    return (os);
}

PointCovariance::PointCovariance(const _PointCovariance& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    covariance_xx = p.covariance_xx;
    covariance_xy = p.covariance_xy;
    covariance_xz = p.covariance_xz;
    covariance_yy = p.covariance_yy;
    covariance_yz = p.covariance_yz;
    covariance_zz = p.covariance_zz;
}

PointCovariance::PointCovariance() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    covariance_xx = covariance_xy = covariance_xz = covariance_yy = covariance_yz = covariance_zz = 0.f;
}

std::ostream& operator<<(std::ostream& os, const PointCovariance& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.covariance_xx << ", " << p.covariance_xy << ", "
       << p.covariance_xz << ", " << p.covariance_yy << ", " << p.covariance_yz << ", " << p.covariance_zz << ")";
    return (os);
}

PointNormalCovariance::PointNormalCovariance(const _PointNormalCovariance& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    normal_x = p.normal_x;
    normal_y = p.normal_y;
    normal_z = p.normal_z;
    data_n[3] = 0.0f;
    curvature = p.curvature;
    covariance_xx = p.covariance_xx;
    covariance_xy = p.covariance_xy;
    covariance_xz = p.covariance_xz;
    covariance_yy = p.covariance_yy;
    covariance_yz = p.covariance_yz;
    covariance_zz = p.covariance_zz;
}

PointNormalCovariance::PointNormalCovariance() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    curvature = 0.f;
    covariance_xx = covariance_xy = covariance_xz = covariance_yy = covariance_yz = covariance_zz = 0.f;
}

std::ostream& operator<<(std::ostream& os, const PointNormalCovariance& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2]
       << " - " << p.curvature << " - " << p.covariance_xx << ", " << p.covariance_xy << ", " << p.covariance_xz << ", "
       << p.covariance_yy << ", " << p.covariance_yz << ", " << p.covariance_zz << ")";
    return (os);
}

PointAngleAxis::PointAngleAxis(const _PointAngleAxis& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    rx = p.rx;
    ry = p.ry;
    rz = p.rz;
    data_r[3] = 0.0f;
}

PointAngleAxis::PointAngleAxis(const Eigen::Isometry3f& pose) {
    setPose(pose);
}

PointAngleAxis::PointAngleAxis(const Eigen::Vector3f& position, const Eigen::AngleAxisf& angle_axis) {
    setPose(position, angle_axis);
}

PointAngleAxis::PointAngleAxis(const Eigen::Vector3f& position, const Eigen::Quaternionf& quaternion) {
    setPose(position, quaternion);
}

PointAngleAxis::PointAngleAxis(const Eigen::Translation<float, 3>& position, const Eigen::Quaternionf& quaternion) {
    setPose(position, quaternion);
}

PointAngleAxis::PointAngleAxis(const Eigen::Translation<float, 3>& position, const Eigen::Matrix3f& rotation_matrix) {
    setPose(position, rotation_matrix);
}

PointAngleAxis::PointAngleAxis() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    rx = ry = rz = data_r[3] = 0.0f;
}

std::ostream& operator<<(std::ostream& os, const PointAngleAxis& p) {
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.rx << "," << p.ry << "," << p.rz << ")";
    return (os);
}
