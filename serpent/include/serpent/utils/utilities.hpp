#ifndef SERPENT_UTILITIES_HPP
#define SERPENT_UTILITIES_HPP

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <ros/time.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <deque>
#include <eigen_ros/imu.hpp>
#include <functional>
#include <mutex>
#include <vector>

#include "serpent/ImuBiases.h"

namespace serpent {

bool check_valid(const gtsam::PreintegrationParams& params);

/**
 * @brief Remove IMU messages strictly older than timestamp (imu.timestamp < timestamp). Buffer is assumed ordered.
 *
 * @param timestamp
 * @param messages
 */
void delete_old_measurements(const ros::Time& timestamp, std::deque<eigen_ros::Imu>& imu_buffer);

/**
 * @brief IMU preintegration assumes that the acceleration and angular velocity remain constant between t and t + dt.
 * Thus the buffer must contain at least one IMU measurment with timestamp <= start. Messages with timestamp > end are
 * ignored.
 *
 * @param preint
 * @param buffer
 * @param start
 * @param end
 */
void integrate_imu(gtsam::PreintegratedCombinedMeasurements& preint, const std::deque<eigen_ros::Imu>& buffer,
        const ros::Time& start, const ros::Time& end);

/**
 * @brief Interpolate between two scalars.
 *
 * @tparam Scalar floating-point type
 * @param value1
 * @param value2
 * @param time1
 * @param time2
 * @param interpolation_time
 * @return Scalar
 */
template<typename Scalar>
Scalar interpolate(const Scalar value1, const Scalar value2, const ros::Time& time1, const ros::Time& time2,
        const ros::Time& interpolation_time);

/**
 * @brief Interpolate a new pressure measurement between between two at a specified timestamp, which must lie between
 * the timestamps of the two measurements. The frame_id of the first pressure measurement is used. The variance is also
 * interpolated.
 *
 * @param pressure1
 * @param pressure2
 * @param interpolation_time
 * @return sensor_msgs::FluidPressure
 */
sensor_msgs::FluidPressure interpolate_pressure(const sensor_msgs::FluidPressure& pressure1,
        const sensor_msgs::FluidPressure& pressure2, const ros::Time& interpolation_time);

/**
 * @brief Sleep while condition is true, and ROS is ok(). The latter check is necessary so that ROS can exit
 * cleanly.
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
std::string to_flat_string(const typename Eigen::Matrix<Scalar, Size, 1>& vector);

void update_preintegration_params(gtsam::PreintegrationParams& params, const Eigen::Matrix3d& accelerometer_covariance,
        const Eigen::Matrix3d& gyroscope_covariance);

}

#include "serpent/utils/utilities-inl.hpp"

#endif
