#include "test_instances.hpp"

#include <string>

pcl::PCLHeader test_header(const unsigned int i) {
    pcl::PCLHeader header;
    header.seq = static_cast<std::uint64_t>(i);
    header.stamp = static_cast<std::uint64_t>(i);
    header.frame_id = std::to_string(i);
    return header;
}

pcl::PointXYZ test_x(const float x) {
    return pcl::PointXYZ(x, 0.f, 0.f);
}

pcl::PointXYZ test_y(const float y) {
    return pcl::PointXYZ(0.f, y, 0.f);
}

pcl::PointXYZ test_z(const float z) {
    return pcl::PointXYZ(0.f, 0.f, z);
}

pcl::PointXYZ test_xyz(const float xyz) {
    return pcl::PointXYZ(xyz, xyz, xyz);
}

pcl::PointNormal test_pn_x(const unsigned int i) {
    pcl::PointNormal p;
    p.x = static_cast<float>(i);
    p.y = 0.f;
    p.z = 0.f;
    p.normal_x = 1.f;
    p.normal_y = 0.f;
    p.normal_z = 0.f;
    p.curvature = 0.f;
    return p;
}

pcl::PointNormal test_pn_y(const unsigned int i) {
    pcl::PointNormal p;
    p.x = 0.f;
    p.y = static_cast<float>(i);
    p.z = 0.f;
    p.normal_x = 0.f;
    p.normal_y = 1.f;
    p.normal_z = 0.f;
    p.curvature = 0.f;
    return p;
}

pcl::PointNormal test_pn_z(const unsigned int i) {
    pcl::PointNormal p;
    p.x = 0.f;
    p.y = 0.f;
    p.z = static_cast<float>(i);
    p.normal_x = 0.f;
    p.normal_y = 0.f;
    p.normal_z = 1.f;
    p.curvature = 0.f;
    return p;
}

pcl::PointNormal test_pn_xyz(const unsigned int i) {
    pcl::PointNormal p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = static_cast<float>(i);
    p.normal_x = 1.f / std::sqrt(3.f);
    p.normal_y = 1.f / std::sqrt(3.f);
    p.normal_z = 1.f / std::sqrt(3.f);
    p.curvature = 0.f;
    return p;
}


PointNormalUnit test_pnu_x(const unsigned int i) {
    PointNormalUnit p;
    p.x = static_cast<float>(i);
    p.y = 0.f;
    p.z = 0.f;
    p.normal_x = 1.f;
    p.normal_y = 0.f;
    p.normal_z = 0.f;
    p.curvature = 0.f;
    p.ux = 1.f;
    p.uy = 0.f;
    p.uz = 0.f;
    return p;
}

PointNormalUnit test_pnu_y(const unsigned int i) {
    PointNormalUnit p;
    p.x = 0.f;
    p.y = static_cast<float>(i);
    p.z = 0.f;
    p.normal_x = 0.f;
    p.normal_y = 1.f;
    p.normal_z = 0.f;
    p.curvature = 0.f;
    p.ux = 0.f;
    p.uy = 1.f;
    p.uz = 0.f;
    return p;
}

PointNormalUnit test_pnu_z(const unsigned int i) {
    PointNormalUnit p;
    p.x = 0.f;
    p.y = 0.f;
    p.z = static_cast<float>(i);
    p.normal_x = 0.f;
    p.normal_y = 0.f;
    p.normal_z = 1.f;
    p.curvature = 0.f;
    p.ux = 0.f;
    p.uy = 0.f;
    p.uz = 1.f;
    return p;
}

PointNormalUnit test_pnu_xyz(const unsigned int i) {
    PointNormalUnit p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = static_cast<float>(i);
    p.normal_x = 1.f / std::sqrt(3.f);
    p.normal_y = 1.f / std::sqrt(3.f);
    p.normal_z = 1.f / std::sqrt(3.f);
    p.curvature = 0.f;
    p.ux = 1.f / std::sqrt(3.f);
    p.uy = 1.f / std::sqrt(3.f);
    p.uz = 1.f / std::sqrt(3.f);
    return p;
}
