#ifndef EIGEN_ROS_NAV_MSGS_HPP
#define EIGEN_ROS_NAV_MSGS_HPP

#include "eigen_ros/odometry.hpp"
#include <nav_msgs/Odometry.h>

namespace eigen_ros {

void from_ros(const nav_msgs::Odometry& msg, Odometry& odom);

void to_ros(nav_msgs::Odometry& msg, const Odometry& odom);

}

#endif
