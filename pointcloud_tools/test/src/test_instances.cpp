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
