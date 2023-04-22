#include "serpent/utilities.hpp"

#include <eigen_ext/covariance.hpp>
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

sensor_msgs::FluidPressure interpolate_pressure(const sensor_msgs::FluidPressure& pressure1,
        const sensor_msgs::FluidPressure& pressure2, const ros::Time& interpolation_time) {
    sensor_msgs::FluidPressure interpolated_pressure;
    interpolated_pressure.header.stamp = interpolation_time;
    interpolated_pressure.header.frame_id = pressure1.header.frame_id;
    interpolated_pressure.fluid_pressure = interpolate(pressure1.fluid_pressure, pressure2.fluid_pressure,
            pressure1.header.stamp, pressure2.header.stamp, interpolation_time);
    interpolated_pressure.variance = interpolate(pressure1.variance, pressure2.variance, pressure1.header.stamp,
            pressure2.header.stamp, interpolation_time);
    return interpolated_pressure;
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
        ROS_ERROR_STREAM("Accelerometer covariance:\n" << accelerometer_covariance);
        ROS_ERROR_STREAM("Gyroscope covariance:\n" << gyroscope_covariance);
        throw std::runtime_error("IMU preintegration parameters were not valid. Covariances with zero diagonal elements"
                                 " can cause this.");
    }
}

}
