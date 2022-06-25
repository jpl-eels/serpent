#ifndef EIGEN_ROS_POSE_HPP
#define EIGEN_ROS_POSE_HPP

#include "eigen_ros/stamped.hpp"
#include <Eigen/Geometry>

namespace eigen_ros {

class Pose {
public:
    Pose(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(), const Eigen::Quaterniond& orientation = 
            Eigen::Quaterniond(1, 0, 0, 0), const Eigen::Matrix<double, 6, 6>& covariance =
            Eigen::Matrix<double, 6, 6>::Zero());
    
    Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
            const Eigen::Matrix3d& position_covariance, const Eigen::Matrix3d& orientation_covariance);

    Pose(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance =
            Eigen::Matrix<double, 6, 6>::Zero());

    // Position
    Eigen::Vector3d position;
    // Orientation
    Eigen::Quaterniond orientation;
    // Covariance matrix with order [R1, R2, R3, t1, t2, t3], i.e. orientation (rad), position (m)
    Eigen::Matrix<double, 6, 6> covariance;
};

using PoseStamped = Stamped<Pose>;

bool operator==(const Pose& lhs, const Pose& rhs);

std::ostream& operator<<(std::ostream& os, const Pose& pose);

/**
 * @brief Apply a transform to the current pose.
 * 
 * TODO FIX: The covariance is currently copied from the current pose, but should be concatenated correctly.
 * 
 * @param current_pose 
 * @param transform 
 * @return Pose 
 */
Pose apply_transform(const Pose& current_pose, const Pose& transform);

/**
 * @brief Obtain the pose in the form of a isometry transform (the covariance is ignored).
 * 
 * @param pose 
 * @return Eigen::Isometry3d 
 */
Eigen::Isometry3d to_transform(const Pose& pose);

}

#endif
