#ifndef EIGEN_ROS_TWIST_HPP
#define EIGEN_ROS_TWIST_HPP

#include <Eigen/Core>

namespace eigen_ros {

class Twist {
public:
    Twist(const Eigen::Vector3d& linear = Eigen::Vector3d::Zero(), const Eigen::Vector3d& angular =
            Eigen::Vector3d::Zero(), const Eigen::Matrix<double, 6, 6>& covariance =
            Eigen::Matrix<double, 6, 6>::Zero());
    
    // Linear velocity
    Eigen::Vector3d linear;
    // Angular velocity
    Eigen::Vector3d angular;
    // Covariance. Order is linear velocity then angular velocity
    Eigen::Matrix<double, 6, 6> covariance;
};

bool operator==(const Twist& lhs, const Twist& rhs);

}

#endif
