#include "serpent/utils/ros_conversion.hpp"

#include <eigen_ros/eigen_ros.hpp>

namespace serpent {

void from_ros(const serpent::ImuBiases& msg, gtsam::imuBias::ConstantBias& imu_biases) {
    imu_biases = gtsam::imuBias::ConstantBias{eigen_ros::from_ros<Eigen::Vector3d>(msg.accelerometer_bias),
            eigen_ros::from_ros<Eigen::Vector3d>(msg.gyroscope_bias)};
}

void from_ros(const std::vector<sensor_msgs::Imu>& msgs, std::deque<eigen_ros::Imu>& imu_array) {
    for (const auto& msg : msgs) {
        imu_array.emplace_back(eigen_ros::from_ros<eigen_ros::Imu>(msg));
    }
}

void from_ros(const geometry_msgs::Pose& msg, gtsam::Pose3& pose) {
    eigen_ros::Pose pose_ = eigen_ros::from_ros<eigen_ros::Pose>(msg);
    pose = gtsam::Pose3(gtsam::Rot3(pose_.orientation), pose_.position);
}

void to_ros(serpent::ImuBiases& msg, const gtsam::imuBias::ConstantBias& imu_biases, const ros::Time& timestamp,
        const std::string& frame_id) {
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
    eigen_ros::to_ros(msg.accelerometer_bias, imu_biases.accelerometer());
    eigen_ros::to_ros(msg.gyroscope_bias, imu_biases.gyroscope());
}

void to_ros(std::vector<sensor_msgs::Imu>& msgs, const std::deque<eigen_ros::Imu>& imu_array) {
    for (const auto& imu : imu_array) {
        msgs.emplace_back(eigen_ros::to_ros<sensor_msgs::Imu>(imu));
    }
}

void to_ros(geometry_msgs::Pose& msg, const gtsam::Pose3& pose) {
    eigen_ros::to_ros(msg.position, pose.translation());
    eigen_ros::to_ros(msg.orientation, pose.rotation().toQuaternion());
}

}
