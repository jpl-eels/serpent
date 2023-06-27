#include "serpent/front_end/stereo/sensor_point_types.hpp"

#include <pcl/PCLPointCloud2.h>

#include <pointcloud_tools/pclpointcloud2_utilities.hpp>

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

}
