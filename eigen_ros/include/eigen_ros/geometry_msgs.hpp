#ifndef EIGEN_ROS_GEOMETRY_MSGS_HPP
#define EIGEN_ROS_GEOMETRY_MSGS_HPP

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>

#include "eigen_ros/pose.hpp"
#include "eigen_ros/twist.hpp"

namespace eigen_ros {

void from_ros(const geometry_msgs::Point& msg, Eigen::Vector3d& v);

void to_ros(geometry_msgs::Point& msg, const Eigen::Vector3d& v);

void from_ros(const geometry_msgs::PoseWithCovarianceStamped& msg, PoseStamped& pose);

void to_ros(geometry_msgs::PoseWithCovarianceStamped& msg, const PoseStamped& pose);

void from_ros(const geometry_msgs::PoseWithCovariance& msg, Pose& pose);

void to_ros(geometry_msgs::PoseWithCovariance& msg, const Pose& pose);

void from_ros(const geometry_msgs::PoseStamped& msg, PoseStamped& pose);

void to_ros(geometry_msgs::PoseStamped& msg, const PoseStamped& pose);

void from_ros(const geometry_msgs::Pose& msg, Pose& pose);

void to_ros(geometry_msgs::Pose& msg, const Pose& pose);

void from_ros(const geometry_msgs::Pose& msg, Eigen::Matrix4d& matrix);

void to_ros(geometry_msgs::Pose& msg, const Eigen::Matrix4d& matrix);

void from_ros(const geometry_msgs::TransformStamped& msg, PoseStamped& pose);

void to_ros(geometry_msgs::TransformStamped& msg, const PoseStamped& pose);

void from_ros(const geometry_msgs::Transform& msg, Pose& pose);

void to_ros(geometry_msgs::Transform& msg, const Pose& pose);

void from_ros(const geometry_msgs::Transform& msg, Eigen::Isometry3d& transform);

void to_ros(geometry_msgs::Transform& msg, const Eigen::Isometry3d& transform);

void from_ros(const geometry_msgs::Transform& msg, Eigen::Matrix4d& matrix);

void to_ros(geometry_msgs::Transform& msg, const Eigen::Matrix4d& matrix);

void from_ros(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond& q);

void to_ros(geometry_msgs::Quaternion& msg, const Eigen::Quaterniond& q);

void from_ros(const geometry_msgs::Quaternion& msg, Eigen::Matrix3d& matrix);

void to_ros(geometry_msgs::Quaternion& msg, const Eigen::Matrix3d& matrix);

void from_ros(const geometry_msgs::TwistWithCovariance& msg, Twist& twist);

void to_ros(geometry_msgs::TwistWithCovariance& msg, const Twist& twist);

void from_ros(const geometry_msgs::Twist& msg, Twist& twist);

void to_ros(geometry_msgs::Twist& msg, const Twist& twist);

void from_ros(const geometry_msgs::Vector3& msg, Eigen::Vector3d& v);

void to_ros(geometry_msgs::Vector3& msg, const Eigen::Vector3d& v);

}

#endif
