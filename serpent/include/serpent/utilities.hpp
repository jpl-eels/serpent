#ifndef SERPENT_UTILITIES_HPP
#define SERPENT_UTILITIES_HPP

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <deque>
#include <eigen_ros/imu.hpp>
#include <functional>
#include <mutex>
#include <vector>

#include "serpent/ImuBiases.h"

namespace serpent {

template<typename Derived>
bool check_non_zero_diagonals(const Eigen::MatrixBase<Derived>& matrix) {
    return matrix.diagonal().minCoeff() > 0.0;
}

bool check_valid(const gtsam::PreintegrationParams& params);

void delete_old_messages(const ros::Time& timestamp, std::deque<eigen_ros::Imu>& messages);

void integrate_imu(gtsam::PreintegratedCombinedMeasurements& preint, const std::deque<eigen_ros::Imu>& buffer,
        const ros::Time& start, const ros::Time& end);

std::vector<sensor_msgs::Imu> old_messages_to_ros(const ros::Time& timestamp,
        const std::deque<eigen_ros::Imu>& messages);

/**
 * @brief Sleep while condition is true, and ROS is ok(). The latter check is necessary so that ROS can exit cleanly.
 *
 * @param mutex
 * @param sleep_period
 * @param leave_locked
 * @param condition
 * @return true if ROS is ok() (i.e. condition broke)
 * @return false if ROS is not ok()
 */
bool protected_sleep(std::mutex& mutex, const double sleep_period, const bool already_locked, const bool leave_locked,
        const std::function<bool()>& condition);

template<typename Scalar, int Size>
std::string to_flat_string(const typename Eigen::Matrix<Scalar, Size, 1>& vector) {
    std::stringstream ss;
    for (std::size_t i = 0; i < vector.rows(); ++i) {
        ss << (i > 0 ? " " : "") << vector[i];
    }
    return ss.str();
}

void update_preintegration_params(gtsam::PreintegrationParams& params, const Eigen::Matrix3d& accelerometer_covariance,
        const Eigen::Matrix3d& gyroscope_covariance);

/* Serialisation */

void from_ros(const serpent::ImuBiases& msg, gtsam::imuBias::ConstantBias& imu_biases);

void to_ros(serpent::ImuBiases& msg, const gtsam::imuBias::ConstantBias& imu_biases, const ros::Time& timestamp,
        const std::string& frame_id = std::string());

void from_ros(const std::vector<sensor_msgs::Imu>& msgs, std::deque<eigen_ros::Imu>& imu_array);

void to_ros(std::vector<sensor_msgs::Imu>& msgs, const std::deque<eigen_ros::Imu>& imu_array);

void from_ros(const geometry_msgs::Pose& msg, gtsam::Pose3& pose);

void to_ros(geometry_msgs::Pose& msg, const gtsam::Pose3& pose);

}

#endif
