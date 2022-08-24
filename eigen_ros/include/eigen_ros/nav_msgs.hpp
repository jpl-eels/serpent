#ifndef EIGEN_ROS_NAV_MSGS_HPP
#define EIGEN_ROS_NAV_MSGS_HPP

#include <nav_msgs/Odometry.h>

#include "eigen_ros/odometry.hpp"

namespace eigen_ros {

void from_ros(const nav_msgs::Odometry& msg, Odometry& odom);

void to_ros(nav_msgs::Odometry& msg, const Odometry& odom);

}

#endif
