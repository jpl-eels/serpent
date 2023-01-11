#ifndef EIGEN_ROS_POSE_HPP
#define EIGEN_ROS_POSE_HPP

#include <Eigen/Geometry>

#include "eigen_ros/stamped.hpp"

namespace eigen_ros {

class Pose {
public:
    Pose(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(),
            const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1, 0, 0, 0),
            const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Zero());

    Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
            const Eigen::Matrix3d& position_covariance, const Eigen::Matrix3d& orientation_covariance);

    Pose(const Eigen::Isometry3d& pose,
            const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Zero());

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
 * @brief Apply a transform to the current pose: T_A^C = T_A^B * T_B^C where T_A^B is the current pose and T_B^C is the
 * transform. The covariance is also transformed from frame B -> C.
 *
 * TODO FIX: The covariance of the second pose is not used (assumed 0), but should be concatenated correctly.
 *
 * @param current_pose
 * @param transform
 * @return Pose
 */
Pose apply_transform(const Pose& current_pose, const Pose& transform);

/**
 * @brief Apply a transform to the current pose: T_A^C = T_A^B * T_B^C where T_A^B is the current pose and T_B^C is the
 * transform. The covariance is also transformed from frame B -> C.
 *
 * @param current_pose
 * @param transform
 * @return Pose
 */
Pose apply_transform(const Pose& current_pose, const Eigen::Isometry3d& transform);

/**
 * @brief Same as apply_transform(const Pose&, const Eigen::Isometry3d&) except for PoseStamped.
 * 
 * @param current_pose 
 * @param transform 
 * @return PoseStamped 
 */
PoseStamped apply_transform(const PoseStamped& current_pose, const Eigen::Isometry3d& transform);

/**
 * @brief Obtain the pose in the form of a isometry transform (the covariance is not used).
 *
 * @param pose
 * @return Eigen::Isometry3d
 */
Eigen::Isometry3d to_transform(const Pose& pose);

}

#endif
