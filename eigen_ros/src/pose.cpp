#include "eigen_ros/pose.hpp"

#include <eigen_ext/geometry.hpp>

namespace eigen_ros {

Pose::Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
        const Eigen::Matrix<double, 6, 6>& covariance)
    : position(position),
      orientation(orientation),
      covariance(covariance) {}

Pose::Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
        const Eigen::Matrix3d& position_covariance, const Eigen::Matrix3d& orientation_covariance)
    : position(position),
      orientation(orientation) {
    covariance << orientation_covariance, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), position_covariance;
}

Pose::Pose(const Eigen::Isometry3d& pose, const Eigen::Matrix<double, 6, 6>& covariance)
    : Pose(pose.translation(), Eigen::Quaterniond{pose.rotation()}, covariance) {}

// We convert the quaternion to a rotation matrix first to avoid the q == -q issue
bool operator==(const Pose& lhs, const Pose& rhs) {
    return lhs.position.isApprox(rhs.position) &&
           lhs.orientation.toRotationMatrix().isApprox(rhs.orientation.toRotationMatrix()) &&
           lhs.covariance.isApprox(rhs.covariance);
}

std::ostream& operator<<(std::ostream& os, const Pose& pose) {
    os << "position (xyz): " << pose.position[0] << ", " << pose.position[1] << ", " << pose.position[2]
       << "\norientation (wxyz): " << pose.orientation.w() << ", " << pose.orientation.x() << ", "
       << pose.orientation.y() << ", " << pose.orientation.z() << "\ncovariance:\n"
       << pose.covariance;
    return os;
}

Pose apply_transform(const Pose& current_pose, const Pose& transform) {
    return apply_transform(current_pose, to_transform(transform));
}

Pose apply_transform(const Pose& current_pose, const Eigen::Isometry3d& transform) {
    return Pose(to_transform(current_pose) * transform,
            eigen_ext::change_covariance_frame(current_pose.covariance, transform.inverse()));
}

PoseStamped apply_transform(const PoseStamped& current_pose, const Eigen::Isometry3d& transform) {
    return PoseStamped{apply_transform(current_pose.data, transform), current_pose.timestamp};
}

Eigen::Isometry3d to_transform(const Pose& pose) {
    return eigen_ext::to_transform(pose.position, pose.orientation);
}

}
