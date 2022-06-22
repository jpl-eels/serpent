#include "test/test_utils.hpp"

std::vector<eigen_ros::Imu> from_ros(const std::vector<sensor_msgs::Imu::ConstPtr>& imu_measurements_ros) {
    std::vector<eigen_ros::Imu> imu_measurements;
    for (const auto& imu_ros : imu_measurements_ros) {
        imu_measurements.emplace_back(eigen_ros::from_ros<eigen_ros::Imu>(*imu_ros));
    }
    return imu_measurements;
}
