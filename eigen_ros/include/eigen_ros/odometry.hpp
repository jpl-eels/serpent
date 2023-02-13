#ifndef EIGEN_ROS_ODOMETRY_HPP
#define EIGEN_ROS_ODOMETRY_HPP

#include <ros/time.h>

#include <Eigen/Geometry>
#include <string>

#include "eigen_ros/pose.hpp"
#include "eigen_ros/twist.hpp"

namespace eigen_ros {

class Odometry {
public:
    Odometry(const Pose& pose = Pose(), const Twist& twist = Twist(), const ros::Time& timestamp = ros::Time(0.0),
            const std::string& frame = std::string(), const std::string& child_frame = std::string());
    
    PoseStamped pose_stamped() const;

    TwistStamped twist_stamped() const;

    // Pose in frame
    Pose pose;
    // Linear and angular velocity in child_frame
    Twist twist;
    // Timestamp
    ros::Time timestamp;
    // Reference frame for transformation
    std::string frame;
    // Name of transformed frame
    std::string child_frame;
};

/**
 * @brief Apply a transform T_{B_i-1}^{B_i} to an odometry O_{A}^{B_i-1} (here A = some fixed frame, B_i-1 is the
 * current odometry child frame, and B_i is the next odometry child frame).
 *
 * TODO FIX: Currently twist is copied across from current odometry, i.e. assumed no change in body twist. Also the
 * child frame name is not changed. Also the covariance is not handled properly.
 *
 * @param current_odometry
 * @param transform
 * @return Odometry
 */
Odometry apply_transform(const Odometry& current_odometry, const PoseStamped& transform);

bool operator==(const Odometry& lhs, const Odometry& rhs);

}

#endif
