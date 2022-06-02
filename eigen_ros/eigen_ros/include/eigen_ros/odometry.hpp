#ifndef EIGEN_ROS_ODOMETRY_HPP
#define EIGEN_ROS_ODOMETRY_HPP

#include "eigen_ros/pose.hpp"
#include "eigen_ros/twist.hpp"
#include <Eigen/Geometry>
#include <ros/time.h>
#include <string>

namespace eigen_ros {

class Odometry {
public:
    Odometry(const Pose& pose = Pose(), const Twist& twist = Twist(), const ros::Time& timestamp = ros::Time(0.0),
            const std::string& frame = std::string(), const std::string& child_frame = std::string());

    Pose pose;
    Twist twist;
    ros::Time timestamp;
    // Reference frame for transformation
    std::string frame;
    // Name of transformed frame
    std::string child_frame;
};

bool operator==(const Odometry& lhs, const Odometry& rhs);

}

#endif
