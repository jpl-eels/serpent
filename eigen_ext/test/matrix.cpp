#include "eigen_ext/matrix.hpp"

#include <gtest/gtest.h>

TEST(skew_symmetric, vector3d_0) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    EXPECT_TRUE(eigen_ext::skew_symmetric(v).isApprox(Eigen::Matrix3d::Zero()));
}

TEST(skew_symmetric, vector3d_1) {
    Eigen::Vector3d v{1.0, 2.0, 3.0};
    Eigen::Matrix3d m;
    m << 0, -3, 2, 3, 0, -1, -2, 1, 0;
    EXPECT_TRUE(eigen_ext::skew_symmetric(v).isApprox(m));
}

TEST(from_skew_symmetric, vector3d_0) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
    EXPECT_TRUE(eigen_ext::from_skew_symmetric(m).isApprox(Eigen::Vector3d::Zero()));
}

TEST(from_skew_symmetric, vector3d_1) {
    Eigen::Vector3d v{1.0, 2.0, 3.0};
    Eigen::Matrix3d m;
    m << 0, -3, 2, 3, 0, -1, -2, 1, 0;
    EXPECT_TRUE(eigen_ext::from_skew_symmetric(m).isApprox(v));
}

TEST(linear_interpolate, vector3d_0) {
    Eigen::Vector3d v1{0, 0, 0};
    Eigen::Vector3d v2{-1, 2, 3.6};
    Eigen::Vector3d v_interp = eigen_ext::linear_interpolate(v1, v2, 0.5);
    EXPECT_TRUE(v_interp.isApprox(v2 * 0.5));
}

TEST(linear_interpolate, vector3d_1) {
    Eigen::Vector3d v1{1, 2, 3};
    Eigen::Vector3d v2{2, 6, 18};
    Eigen::Vector3d v_interp = eigen_ext::linear_interpolate(v1, v2, 0.5);
    EXPECT_TRUE(v_interp.isApprox(Eigen::Vector3d{1.5, 4, 10.5}));
}

TEST(linear_interpolate, vector3d_2) {
    Eigen::Vector3d v1{1, 2, 3};
    Eigen::Vector3d v2{4, 5, 6};
    Eigen::Vector3d v_interp = eigen_ext::linear_interpolate(v1, v2, 0);
    EXPECT_TRUE(v_interp.isApprox(v1));
}

TEST(linear_interpolate, vector3d_3) {
    Eigen::Vector3d v1{1, 2, 3};
    Eigen::Vector3d v2{4, 5, 6};
    Eigen::Vector3d v_interp = eigen_ext::linear_interpolate(v1, v2, 1);
    EXPECT_TRUE(v_interp.isApprox(v2));
}

