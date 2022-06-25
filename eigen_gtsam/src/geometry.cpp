#include "eigen_gtsam/geometry.hpp"

namespace eigen_gtsam {

void to_eigen(const gtsam::Pose3& from, Eigen::Isometry3d& to) {
    to = Eigen::Isometry3d{from.matrix()};
}

void to_eigen(const gtsam::Rot3& from, Eigen::Matrix3d& to) {
    to = from.matrix();
}

void to_gtsam(const Eigen::Isometry3d& from, gtsam::Pose3& to) {
    to = gtsam::Pose3{from.matrix()};
}

}
