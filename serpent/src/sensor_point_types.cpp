#include "serpent/sensor_point_types.hpp"
#include <pointcloud_tools/pclpointcloud2_utilities.hpp>
#include <pcl/PCLPointCloud2.h>

namespace serpent {

SensorType to_sensor_type(const std::string& sensor_type) {
    if (sensor_type == "OUSTER") {
        return SensorType::OUSTER;
    } else if (sensor_type == "CUSTOM") {
        return SensorType::CUSTOM;
    }
    throw std::runtime_error("Sensor type could not be converted from string \'" + sensor_type + "\'");
}

pcl::PCLPointField& time_field(pcl::PCLPointCloud2& pointcloud, const SensorType sensor_type) {
    switch (sensor_type) {
        case SensorType::OUSTER:
            return pct::get_field(pointcloud, "t");
        case SensorType::CUSTOM:
            return pct::get_field(pointcloud, "t");
        default:
            throw std::runtime_error("Sensor type not handled. Cannot determine time field.");
    }
}

void ouster_ns_to_s(pcl::PCLPointCloud2& pointcloud) {
    auto& t_field = time_field(pointcloud, SensorType::OUSTER);
    if (t_field.datatype != pcl::PCLPointField::PointFieldTypes::UINT32) {
        throw std::runtime_error("Expected time field t to be uint32");
    }
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        std::uint32_t* t = reinterpret_cast<std::uint32_t*>(&pointcloud.data[i + t_field.offset]);
        float t_ = static_cast<double>(*t) / 1.0e9;
        *reinterpret_cast<float*>(t) = t_;
    }
    t_field.datatype = pcl::PCLPointField::PointFieldTypes::FLOAT32;
}

}

/*
namespace pcl {

template<>
void copyPointCloud(const pcl::PointCloud<PointOuster>& src, pcl::PointCloud<PointXYZT>& dest) {
    dest.header = src.header;
    dest.resize(src.size());
    dest.width = src.width;
    dest.height = src.height;
    dest.is_dense = src.is_dense;
    dest.sensor_origin_ = src.sensor_origin_;
    dest.sensor_orientation_ = src.sensor_orientation_;
    for (std::size_t i = 0; i < src.size(); ++i) {
        dest.points[i].x = src.points[i].x;
        dest.points[i].y = src.points[i].y;
        dest.points[i].z = src.points[i].z;
        dest.points[i].t = src.points[i].t;
    }
}

}
*/
