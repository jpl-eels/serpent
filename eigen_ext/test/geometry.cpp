#include "eigen_ext/geometry.hpp"
#include "eigen_ext/matrix.hpp"
#include <gtest/gtest.h>

void check_transform_adjoint_blocks(const Eigen::Matrix<double, 6, 6>& transform_adjoint,
        const Eigen::Isometry3d& transform) {
    const Eigen::Matrix3d R = transform.rotation();
    const Eigen::Vector3d t = transform.translation();
    const Eigen::Matrix3d t_SS = eigen_ext::skew_symmetric(t);
    const Eigen::Matrix3d t_SS_times_R = t_SS * R;
    const Eigen::Matrix3d top_left = transform_adjoint.block<3, 3>(0, 0);
    const Eigen::Matrix3d top_right = transform_adjoint.block<3, 3>(0, 3);
    const Eigen::Matrix3d bottom_left = transform_adjoint.block<3, 3>(3, 0);
    const Eigen::Matrix3d bottom_right = transform_adjoint.block<3, 3>(3, 3);
    EXPECT_TRUE(top_left.isApprox(R));
    EXPECT_TRUE(top_right.isApprox(Eigen::Matrix3d::Zero()));
    EXPECT_TRUE(bottom_left.isApprox(t_SS_times_R));
    EXPECT_TRUE(bottom_right.isApprox(R));
}

TEST(transform_adjoint, rotation_only) {
    const Eigen::Isometry3d transform{Eigen::Quaterniond(0.17, 0.68, 0.55, 0.14).normalized()};
    Eigen::Matrix<double, 6, 6> transform_adjoint = eigen_ext::transform_adjoint(transform);
    check_transform_adjoint_blocks(transform_adjoint, transform);
}

TEST(transform_adjoint, translation_only) {
    const Eigen::Isometry3d transform{Eigen::Translation<double, 3>{1.0, 2.0, 3.0}};
    Eigen::Matrix<double, 6, 6> transform_adjoint = eigen_ext::transform_adjoint(transform);
    check_transform_adjoint_blocks(transform_adjoint, transform);
}

TEST(transform_adjoint, transform) {
    const Eigen::Isometry3d transform = eigen_ext::to_transform(Eigen::Translation<double, 3>{1.0, 2.0, 3.0},
            Eigen::Quaterniond(0.17, 0.68, 0.55, 0.14).normalized());
    Eigen::Matrix<double, 6, 6> transform_adjoint = eigen_ext::transform_adjoint(transform);
    check_transform_adjoint_blocks(transform_adjoint, transform);
}
