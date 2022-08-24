#ifndef EIGEN_ROS_SENSOR_MSGS_HPP
#define EIGEN_ROS_SENSOR_MSGS_HPP

#include <sensor_msgs/Imu.h>

#include "eigen_ros/imu.hpp"

namespace eigen_ros {

void from_ros(const sensor_msgs::Imu& msg, Imu& imu);

void to_ros(sensor_msgs::Imu& msg, const Imu& imu);

}

#endif
