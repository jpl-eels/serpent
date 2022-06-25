#include "eigen_ext/matrix.hpp"
#include <gtest/gtest.h>

TEST(skew_symmetric, vector3d_0) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    EXPECT_TRUE(eigen_ext::skew_symmetric(v).isApprox(Eigen::Matrix3d::Zero()));
}

TEST(skew_symmetric, vector3d_1) {
    Eigen::Vector3d v{1.0, 2.0, 3.0};
    Eigen::Matrix3d m;
    m <<  0, -3,  2,
          3,  0, -1,
         -2,  1,  0;
    EXPECT_TRUE(eigen_ext::skew_symmetric(v).isApprox(m));
}
