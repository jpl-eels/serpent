#ifndef EIGEN_ROS_TWIST_HPP
#define EIGEN_ROS_TWIST_HPP

#include <Eigen/Core>

#include "eigen_ros/stamped.hpp"

namespace eigen_ros {

class Twist {
public:
    Twist(const Eigen::Vector3d& linear = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& angular = Eigen::Vector3d::Zero(),
            const Eigen::Matrix<double, 6, 6>& covariance = Eigen::Matrix<double, 6, 6>::Zero());

    Twist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular, const Eigen::Matrix3d& linear_covariance,
            const Eigen::Matrix3d& angular_covariance);

    Eigen::Matrix3d angular_velocity_covariance() const;

    Eigen::Matrix3d linear_velocity_covariance() const;

    // Linear velocity
    Eigen::Vector3d linear;
    // Angular velocity
    Eigen::Vector3d angular;
    // Covariance. Order is [w1, w2, w3, v1, v2, v3], i.e. angular velocity then linear velocity
    Eigen::Matrix<double, 6, 6> covariance;
};

using TwistStamped = Stamped<Twist>;

bool operator==(const Twist& lhs, const Twist& rhs);

}

#endif
