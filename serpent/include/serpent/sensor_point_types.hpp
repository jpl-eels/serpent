#ifndef SERPENT_SENSOR_POINT_TYPES_HPP
#define SERPENT_SENSOR_POINT_TYPES_HPP

#include <pcl/PCLPointCloud2.h>

#include <string>

namespace serpent {

enum class SensorType {
    OUSTER,
    CUSTOM
};

/**
 * @brief Convert string to sensor type
 *
 * @param sensor_type
 * @return SensorType
 */
SensorType to_sensor_type(const std::string& sensor_type);

/**
 * @brief Get type field for sensor type
 *
 * @param sensor_type
 * @return const pcl::PCLPointField&
 */
pcl::PCLPointField& time_field(pcl::PCLPointCloud2& pointcloud, const SensorType sensor_type);

}

/*
struct PointOuster {
    PCL_ADD_POINT4D     // float: x, y, z
    PCL_ADD_INTENSITY   // float: i
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
        (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity) (std::uint8_t, ring, ring)
        (std::uint16_t, ambient, ambient) (std::uint32_t, range, range))
*/

#endif
