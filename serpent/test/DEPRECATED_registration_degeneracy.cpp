#include "serpent/DEPRECATED_registration_degeneracy.hpp"

#include <gtest/gtest.h>

#define DOUBLE_PRECISION (1.0e-12)

TEST(point_to_point_degeneracy, jacobian) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};
    const double angle = r.norm();
    const Eigen::AngleAxisd angleaxis{angle, r.normalized()};
    const Eigen::Matrix3d R = angleaxis.toRotationMatrix();
    const double sina = std::sin(angle);
    const double cosa = std::cos(angle);
    const Eigen::Matrix<double, 6, 1> jacobian = serpent::point_to_point_jacobian(R, t, r, sina, cosa, p, q);
    Eigen::Matrix<double, 6, 1> matlab_jacobian;
    matlab_jacobian << -58.076236734788900, -35.728638292524053, 34.514473846887434, -18.821961096383522,
            13.702096633764571, 32.411231516355087;
    EXPECT_NEAR(jacobian[0], matlab_jacobian[0], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[1], matlab_jacobian[1], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[2], matlab_jacobian[2], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[3], matlab_jacobian[3], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[4], matlab_jacobian[4], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[5], matlab_jacobian[5], DOUBLE_PRECISION);
}

TEST(point_to_plane_degeneracy, jacobian) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};
    const Eigen::Vector3d n{0.455842305838552, 0.683763458757828, 0.569802882298190};

    const Eigen::Matrix<double, 6, 1> jacobian = serpent::point_to_plane_jacobian(t, r, p, q, n);
    Eigen::Matrix<double, 6, 1> matlab_jacobian;
    matlab_jacobian << 3.836913399274006, 24.066190794285660, -16.139797708030649, 8.778228207731365,
            13.167342311597046, 10.972785259664207;
    EXPECT_NEAR(jacobian[0], matlab_jacobian[0], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[1], matlab_jacobian[1], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[2], matlab_jacobian[2], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[3], matlab_jacobian[3], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[4], matlab_jacobian[4], DOUBLE_PRECISION);
    EXPECT_NEAR(jacobian[5], matlab_jacobian[5], DOUBLE_PRECISION);
}
