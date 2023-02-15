#include <pointcloud_tools/point_types.hpp>

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
