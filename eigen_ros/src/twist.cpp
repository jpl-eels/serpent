#include "eigen_ros/twist.hpp"

namespace eigen_ros {

Twist::Twist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular,
        const Eigen::Matrix<double, 6, 6>& covariance)
    : linear(linear),
      angular(angular),
      covariance(covariance) {}

Twist::Twist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular, const Eigen::Matrix3d& linear_covariance,
        const Eigen::Matrix3d& angular_covariance)
    : linear(linear),
      angular(angular) {
    covariance << angular_covariance, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), linear_covariance;
}

bool operator==(const Twist& lhs, const Twist& rhs) {
    return lhs.linear.isApprox(rhs.linear) && lhs.angular.isApprox(rhs.angular) &&
           lhs.covariance.isApprox(rhs.covariance);
}

Eigen::Matrix3d Twist::angular_velocity_covariance() const {
    return covariance.block<3, 3>(0, 0);
}

Eigen::Matrix3d Twist::linear_velocity_covariance() const {
    return covariance.block<3, 3>(3, 3);
}

}
