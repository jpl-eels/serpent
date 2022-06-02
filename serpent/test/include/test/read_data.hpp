#ifndef SERPENT_TEST_READ_DATA_HPP
#define SERPENT_TEST_READ_DATA_HPP

#include <sensor_msgs/Imu.h>

std::vector<sensor_msgs::Imu::ConstPtr> read_imu(const std::string& bagfile, const std::string& topic);

#endif
