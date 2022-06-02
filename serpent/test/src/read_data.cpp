#include "test/read_data.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>

std::vector<sensor_msgs::Imu::ConstPtr> read_imu(const std::string& bagfile, const std::string& topic) {
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    std::vector<sensor_msgs::Imu::ConstPtr> imu_measurements;
    for (const rosbag::MessageInstance& msg : view) {
        imu_measurements.emplace_back(msg.instantiate<sensor_msgs::Imu>());
        if (!imu_measurements.back()) {
            throw std::runtime_error("IMU measurement was nullptr");
        }
    }
    return imu_measurements;
}
