#ifndef SERPENT_POINTCLOUD_FORMATTER_HPP
#define SERPENT_POINTCLOUD_FORMATTER_HPP

#include "serpent/sensor_point_types.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace serpent {

class PointcloudFormatter {
public:
    explicit PointcloudFormatter();

private:
    void format(const sensor_msgs::PointCloud2::ConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Publisher pointcloud_publisher;
    ros::Subscriber pointcloud_subscriber;

    //// Configuration
    // Sensor type for sensor-specific processing
    SensorType sensor_type;
    // Time field enabled
    bool time_field_filter_enabled;
    float max_time_threshold;
};

}

#endif
