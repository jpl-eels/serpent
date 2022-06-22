#ifndef SERPENT_TEST_TEST_UTILS_HPP
#define SERPENT_TEST_TEST_UTILS_HPP

#include <eigen_ros/eigen_ros.hpp>
#include <sensor_msgs/Imu.h>
#include <vector>

std::vector<eigen_ros::Imu> from_ros(const std::vector<sensor_msgs::Imu::ConstPtr>& imu_measurements_ros);

#endif
