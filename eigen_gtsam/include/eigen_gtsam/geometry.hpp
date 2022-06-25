#ifndef EIGEN_GTSAM_GEOMETRY_HPP
#define EIGEN_GTSAM_GEOMETRY_HPP

#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace eigen_gtsam {

void to_eigen(const gtsam::Pose3& from, Eigen::Isometry3d& to);

void to_eigen(const gtsam::Rot3& from, Eigen::Matrix3d& to);

void to_gtsam(const Eigen::Isometry3d& from, gtsam::Pose3& to);

template<typename Derived>
void to_gtsam(const Eigen::MatrixBase<Derived>& from, gtsam::Rot3& to);

/* Implementation */

template<typename Derived>
void to_gtsam(const Eigen::MatrixBase<Derived>& from, gtsam::Rot3& to) {
    to = gtsam::Rot3{from};
}

}

#endif
