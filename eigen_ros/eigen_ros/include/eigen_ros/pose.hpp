#ifndef EIGEN_ROS_POSE_HPP
#define EIGEN_ROS_POSE_HPP

#include "eigen_ros/stamped.hpp"
#include <Eigen/Geometry>

namespace eigen_ros {

class Pose {
public:
    Pose(const Eigen::Vector3d& position = Eigen::Vector3d::Zero(), const Eigen::Quaterniond& orientation = 
            Eigen::Quaterniond(1, 0, 0, 0), const Eigen::Matrix<double, 6, 6> covariance =
            Eigen::Matrix<double, 6, 6>::Zero());

    // Position
    Eigen::Vector3d position;
    // Orientation
    Eigen::Quaterniond orientation;
    // Covariance. Order is position then orientation, i.e. XYZRPY
    Eigen::Matrix<double, 6, 6> covariance;
};

using PoseStamped = Stamped<Pose>;

bool operator==(const Pose& lhs, const Pose& rhs);

std::ostream& operator<<(std::ostream& os, const Pose& pose);

inline Eigen::Isometry3d to_transform(const Pose& pose) {
    return Eigen::Translation<double, 3>(pose.position) * pose.orientation;
}

}

#endif
