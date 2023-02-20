#ifndef SERPENT_ROS_CONVERSION_HPP
#define SERPENT_ROS_CONVERSION_HPP

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <eigen_ros/imu.hpp>
#include <string>
#include <vector>

#include "serpent/ImuBiases.h"

namespace serpent {

void from_ros(const serpent::ImuBiases& msg, gtsam::imuBias::ConstantBias& imu_biases);

void from_ros(const std::vector<sensor_msgs::Imu>& msgs, std::deque<eigen_ros::Imu>& imu_array);

void from_ros(const geometry_msgs::Pose& msg, gtsam::Pose3& pose);

void to_ros(serpent::ImuBiases& msg, const gtsam::imuBias::ConstantBias& imu_biases, const ros::Time& timestamp,
        const std::string& frame_id = std::string());

void to_ros(std::vector<sensor_msgs::Imu>& msgs, const std::deque<eigen_ros::Imu>& imu_array);

void to_ros(geometry_msgs::Pose& msg, const gtsam::Pose3& pose);

}

#endif
