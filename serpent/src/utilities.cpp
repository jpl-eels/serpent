#include "serpent/utilities.hpp"

#include <eigen_ext/covariance.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/sensor_msgs.hpp>

namespace serpent {

bool check_valid(const gtsam::PreintegrationParams& params) {
    return params.getGravity().norm() > 0.0 && eigen_ext::is_valid_covariance(params.getAccelerometerCovariance()) &&
           eigen_ext::is_valid_covariance(params.getGyroscopeCovariance()) &&
           eigen_ext::is_valid_covariance(params.getIntegrationCovariance());
}

void delete_old_measurements(const ros::Time& timestamp, std::deque<eigen_ros::Imu>& imu_buffer) {
    while (!imu_buffer.empty() && imu_buffer.front().timestamp < timestamp) {
        imu_buffer.pop_front();
    }
}

void integrate_imu(gtsam::PreintegratedCombinedMeasurements& preint, const std::deque<eigen_ros::Imu>& buffer,
        const ros::Time& start, const ros::Time& end) {
    // Exit early if no integration required
    if (start == end) {
        return;
    }

    // Error handling
    if (buffer.size() < 2) {
        throw std::runtime_error("IMU buffer must contain at least 2 measurements for valid preintegration.");
    } else if (buffer.front().timestamp > start) {
        throw std::runtime_error("IMU buffer must contain at least one IMU measurment with timestamp <= start for valid"
                                 " preintegration.");
    }

    ros::Time integration_time = start;
    for (auto it = buffer.cbegin(); it != buffer.cend(); ++it) {
        // Ignore imu messages past end
        if (it->timestamp > end) {
            break;
        }
        // If the next timestamp is after start time, we integrate
        auto next_it = std::next(it);
        if (next_it == buffer.cend() || next_it->timestamp > start) {
            // Integrate from the current imu timestamp (integration_time_end) to the next imu timestamp. Account for
            // the cases where the last imu buffer message is before or after end.
            const ros::Time integration_time_end = (next_it == buffer.cend()) ? end : std::min(next_it->timestamp, end);
            const double dt = (integration_time_end - integration_time).toSec();
            integration_time = integration_time_end;
            preint.integrateMeasurement(it->linear_acceleration, it->angular_velocity, dt);
        }
    }
}

Eigen::Matrix<double, 6, 6> reorder_pose_covariance(const Eigen::Matrix<double, 6, 6>& covariance) {
    Eigen::Matrix<double, 6, 6> reordered_covariance;
    reordered_covariance << covariance.block<3, 3>(3, 3), covariance.block<3, 3>(3, 0), covariance.block<3, 3>(0, 3),
            covariance.block<3, 3>(0, 0);
    return reordered_covariance;
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

void update_preintegration_params(gtsam::PreintegrationParams& params, const Eigen::Matrix3d& accelerometer_covariance,
        const Eigen::Matrix3d& gyroscope_covariance) {
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
