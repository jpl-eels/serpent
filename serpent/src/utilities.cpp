#include "serpent/utilities.hpp"
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/sensor_msgs.hpp>
#include <eigen_ros/eigen_ros.hpp>

namespace serpent {

bool check_valid(const gtsam::PreintegrationParams& params) {
    return params.getGravity().norm() > 0.0 && check_non_zero_diagonals(params.getAccelerometerCovariance()) &&
            check_non_zero_diagonals(params.getGyroscopeCovariance()) &&
            check_non_zero_diagonals(params.getIntegrationCovariance());

}

void delete_old_messages(const ros::Time& timestamp, std::deque<eigen_ros::Imu>& messages) {
    while (!messages.empty() && messages.front().timestamp < timestamp) {
        messages.pop_front();
    }
}

Eigen::Matrix<double, 6, 6> reorder_pose_covariance(const Eigen::Matrix<double, 6, 6>& covariance) {
    Eigen::Matrix<double, 6, 6> reordered_covariance;
    reordered_covariance << covariance.block<3, 3>(3, 3), covariance.block<3, 3>(3, 0), covariance.block<3, 3>(0, 3),
            covariance.block<3, 3>(0, 0);
    return reordered_covariance;
}

std::vector<sensor_msgs::Imu> old_messages_to_ros(const ros::Time& timestamp,
        const std::deque<eigen_ros::Imu>& messages)
{
    std::vector<sensor_msgs::Imu> extracted;
    for (const auto& msg : messages) {
        if (msg.timestamp < timestamp) {
            extracted.emplace_back(eigen_ros::to_ros<sensor_msgs::Imu>(msg));
        }
    }
    return extracted;
}

bool protected_sleep(std::mutex& mutex, const double sleep_period_, const bool already_locked, const bool leave_locked,
        const std::function<bool()>& condition) {
    const ros::Duration sleep_period{sleep_period_};
    if (!already_locked) {
        mutex.lock();
    }
    while (ros::ok() && condition()) {
        mutex.unlock();
        sleep_period.sleep();
        mutex.lock();
    }
    if (!leave_locked) {
        mutex.unlock();
    }
    return ros::ok();
}

void update_preintegration_params(gtsam::PreintegrationParams& params,
        const Eigen::Matrix3d& accelerometer_covariance, const Eigen::Matrix3d& gyroscope_covariance) {
    params.setAccelerometerCovariance(accelerometer_covariance);
    params.setGyroscopeCovariance(gyroscope_covariance);
    if (!check_valid(params)) {
        ROS_WARN_STREAM("Accelerometer covariance:\n" << accelerometer_covariance);
        ROS_WARN_STREAM("Gyroscope covariance:\n" << gyroscope_covariance);
        throw std::runtime_error("IMU preintegration parameters were not valid. Covariances with zero diagonal elements"
                " can cause this.");
    }
}

void from_ros(const serpent::ImuBiases& msg, gtsam::imuBias::ConstantBias& imu_biases) {
    imu_biases = gtsam::imuBias::ConstantBias{eigen_ros::from_ros<Eigen::Vector3d>(msg.accelerometer_bias),
            eigen_ros::from_ros<Eigen::Vector3d>(msg.gyroscope_bias)};
}

void to_ros(serpent::ImuBiases& msg, const gtsam::imuBias::ConstantBias& imu_biases, const ros::Time& timestamp,
        const std::string& frame_id) {
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
    eigen_ros::to_ros(msg.accelerometer_bias, imu_biases.accelerometer());
    eigen_ros::to_ros(msg.gyroscope_bias, imu_biases.gyroscope());
}

void from_ros(const std::vector<sensor_msgs::Imu>& msgs, std::deque<eigen_ros::Imu>& imu_array) {
    for (const auto& msg : msgs) {
        imu_array.emplace_back(eigen_ros::from_ros<eigen_ros::Imu>(msg));
    }
}

void to_ros(std::vector<sensor_msgs::Imu>& msgs, const std::deque<eigen_ros::Imu>& imu_array) {
    for (const auto& imu : imu_array) {
        msgs.emplace_back(eigen_ros::to_ros<sensor_msgs::Imu>(imu));
    }
}

void from_ros(const geometry_msgs::Pose& msg, gtsam::Pose3& pose) {
    eigen_ros::Pose pose_ = eigen_ros::from_ros<eigen_ros::Pose>(msg);
    pose = gtsam::Pose3(gtsam::Rot3(pose_.orientation), pose_.position);
}

void to_ros(geometry_msgs::Pose& msg, const gtsam::Pose3& pose) {
    eigen_ros::to_ros(msg.position, pose.translation());
    eigen_ros::to_ros(msg.orientation, pose.rotation().toQuaternion());
}

}
