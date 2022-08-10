#include "serpent/pointcloud_formatter.hpp"
#include <pointcloud_tools/pclpointcloud2_utilities.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

namespace serpent {

PointcloudFormatter::PointcloudFormatter():
    nh("~")
{
    // Publishers
    pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("formatter/formatted_pointcloud", 1);

    // Subscribers
    pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input/pointcloud", 100,
            &PointcloudFormatter::format, this);

    // Configuration
    sensor_type = to_sensor_type(nh.param<std::string>("pointcloud_type", "OUSTER"));
    time_field_filter_enabled = nh.param<bool>("time_field_filter/enabled", false);
    if (time_field_filter_enabled) {
        max_time_threshold = nh.param<float>("time_field_filter/max", 0.1);
    }
}

void PointcloudFormatter::format(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert pointcloud from ROS
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pointcloud);

    // Sensor-specific processing
    switch (sensor_type) {
        case SensorType::OUSTER:
            ouster_ns_to_s(*pointcloud);
            pct::cast_to_float32(*pointcloud, "range");
            pct::scale_float32_field(*pointcloud, "range", 0.001f);
            break;
        case SensorType::CUSTOM:
            if (!pct::has_field(*pointcloud, "t") && pct::has_field(*pointcloud, "time")) {
                pct::change_field_name(*pointcloud, "time", "t");
            }
            break;
        default:
            throw std::runtime_error("Sensor type not valid enum. Something went wrong.");
    }

    // Time field filter
    if (time_field_filter_enabled) {
        // Ensure time field is present
        if (!pct::has_field(*pointcloud, "t")) {
            throw std::runtime_error("Point cloud must t field with time field filter enabled. Disable filter or add t"
                    "field to input point cloud.");
        }

        // Ensure time field "t" is present and is float32
        const auto& t_field = pct::get_field(*pointcloud, "t");
        if (t_field.datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32) {
            throw std::runtime_error("t fields that are not FLOAT32 currently not supported.");
        }

        auto filtered_pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
        pct::filter_max(*pointcloud, *filtered_pointcloud, t_field, max_time_threshold);
        pointcloud = filtered_pointcloud;
    }

    // Change frame
    pointcloud->header.frame_id = "lidar";
    
    // Publish pointcloud
    pointcloud_publisher.publish(pointcloud);
}

}
