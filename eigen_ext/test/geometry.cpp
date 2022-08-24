#include "eigen_ext/geometry.hpp"

#include <gtest/gtest.h>

#include "eigen_ext/matrix.hpp"

#define DOUBLE_PRECISION (1.0e-12)

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

TEST(relative_transform, linear_rates_0) {
    const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d rotation_axis = Eigen::Vector3d(0.8, 0.1, 0.05).normalized();
    const double rotation_angle_rate{0.492};
    const Eigen::Vector3d linear_velocity{1.0, 2.0, 3.0};
    const double dt = 1.0;
    const Eigen::Vector3d angular_velocity = rotation_angle_rate * dt * rotation_axis;
    const Eigen::Isometry3d pose = Eigen::Translation<double, 3>{linear_velocity * dt} *
                                   Eigen::AngleAxisd{rotation_angle_rate * dt, rotation_axis};
    Eigen::Matrix<double, 6, 1> rates = eigen_ext::linear_rates(I, pose, dt);
    const Eigen::Vector3d angular_velocity_out = rates.block<3, 1>(0, 0);
    const Eigen::Vector3d linear_velocity_out = rates.block<3, 1>(3, 0);
    EXPECT_TRUE(angular_velocity_out.isApprox(angular_velocity));
    EXPECT_TRUE(linear_velocity_out.isApprox(linear_velocity));
}

TEST(relative_transform, linear_rates_1) {
    const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d rotation_axis = Eigen::Vector3d(0.8, 0.1, 0.05).normalized();
    const double rotation_angle_rate{0.492};
    const Eigen::Vector3d linear_velocity{1.0, 2.0, 3.0};
    const double dt = M_PI / rotation_angle_rate;  // Too large a dt will invalidate test because the rotation will wrap
    const Eigen::Vector3d angular_velocity = rotation_angle_rate * rotation_axis;
    const Eigen::Isometry3d pose = Eigen::Translation<double, 3>{linear_velocity * dt} *
                                   Eigen::AngleAxisd{rotation_angle_rate * dt, rotation_axis};
    Eigen::Matrix<double, 6, 1> rates = eigen_ext::linear_rates(I, pose, dt);
    const Eigen::Vector3d angular_velocity_out = rates.block<3, 1>(0, 0);
    const Eigen::Vector3d linear_velocity_out = rates.block<3, 1>(3, 0);
    EXPECT_TRUE(angular_velocity_out.isApprox(angular_velocity));
    EXPECT_TRUE(linear_velocity_out.isApprox(linear_velocity));
}

TEST(relative_transform, transform_0) {
    const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d pose =
            Eigen::Translation<double, 3>{1.0, 2.0, 3.0} * Eigen::Quaterniond{0.8, 0.1, 0.05, 0.2}.normalized();
    const Eigen::Isometry3d transform = eigen_ext::relative_transform(I, pose);
    EXPECT_TRUE(transform.isApprox(pose));
}

TEST(relative_transform, transform_0_inv) {
    const Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
    const Eigen::Vector3d translation{1.0, 2.0, 3.0};
    const Eigen::Isometry3d pose =
            Eigen::Translation<double, 3>{translation} * Eigen::Quaterniond{0.8, 0.1, 0.05, 0.2}.normalized();
    const Eigen::Isometry3d transform = eigen_ext::relative_transform(pose, I);
    EXPECT_TRUE(transform.isApprox(pose.inverse()));
}

TEST(change_relative_transform_frame, identity_with_identity) {
    const Eigen::Isometry3d relative_transform_A = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d transform_B_A =
            Eigen::Translation<double, 3>{1.0, 2.0, 3.0} * Eigen::Quaterniond{0.8, 0.1, 0.05, 0.2}.normalized();
    const Eigen::Isometry3d relative_transform_B =
            eigen_ext::change_relative_transform_frame(relative_transform_A, transform_B_A);
    const Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
    EXPECT_TRUE(relative_transform_B.isApprox(expected));
}

TEST(relative_transform_change_frame, identity_with_transform) {
    const Eigen::Isometry3d relative_transform_A = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d transform_B_A = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d relative_transform_B =
            eigen_ext::change_relative_transform_frame(relative_transform_A, transform_B_A);
    const Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
    EXPECT_TRUE(relative_transform_B.isApprox(expected));
}

TEST(change_relative_transform_frame, transform_with_identity) {
    const Eigen::Isometry3d relative_transform_A =
            Eigen::Translation<double, 3>{-5.0, 6.0, -10.0} * Eigen::Quaterniond{0.4, 0.2, 0.7, 0.15}.normalized();
    const Eigen::Isometry3d transform_B_A = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d relative_transform_B =
            eigen_ext::change_relative_transform_frame(relative_transform_A, transform_B_A);
    const Eigen::Isometry3d expected = relative_transform_A;
    EXPECT_TRUE(relative_transform_B.isApprox(expected));
}

TEST(change_relative_transform_frame, translation_with_translation) {
    const Eigen::Isometry3d relative_transform_A{Eigen::Translation<double, 3>{-5.0, 6.0, -10.0}};
    const Eigen::Isometry3d transform_B_A{Eigen::Translation<double, 3>{1.0, 2.0, 3.0}};
    const Eigen::Isometry3d relative_transform_B =
            eigen_ext::change_relative_transform_frame(relative_transform_A, transform_B_A);
    const Eigen::Isometry3d expected = relative_transform_A;
    EXPECT_TRUE(relative_transform_B.isApprox(expected));
}

TEST(change_relative_transform_frame, rotation_with_rotation) {
    const Eigen::Isometry3d relative_transform_A{Eigen::Quaterniond{0.4, 0.2, 0.7, 0.15}.normalized()};
    const Eigen::Isometry3d transform_B_A{Eigen::Quaterniond{0.8, 0.1, 0.05, 0.2}.normalized()};
    const Eigen::Isometry3d relative_transform_B =
            eigen_ext::change_relative_transform_frame(relative_transform_A, transform_B_A);
    EXPECT_NEAR(Eigen::AngleAxisd(relative_transform_B.rotation()).angle(),
            Eigen::AngleAxisd(relative_transform_A.rotation()).angle(), DOUBLE_PRECISION);
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
