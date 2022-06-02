#include "eigen_ros/twist.hpp"

namespace eigen_ros {

Twist::Twist(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular,
        const Eigen::Matrix<double, 6, 6>& covariance):
    linear(linear), angular(angular), covariance(covariance) {}

bool operator==(const Twist& lhs, const Twist& rhs) {
    return lhs.linear.isApprox(rhs.linear) && lhs.angular.isApprox(rhs.angular) &&
            lhs.covariance.isApprox(rhs.covariance);
}

}
