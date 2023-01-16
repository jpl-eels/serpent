#include "test/test_instances.hpp"

#include <eigen_ext/geometry.hpp>

Eigen::Matrix3d test_covariance_3x3(const unsigned int i) {
    Eigen::Matrix3d cov;
    switch (i) {
        case 0:
            cov <<  0.1, 0.0, 0.0,
                    0.0, 0.1, 0.0,
                    0.0, 0.0, 0.1;
            break;
        case 1:
            cov <<  0.1, 0.0, 0.0,
                    0.0, 0.2, 0.0,
                    0.0, 0.0, 0.3;
            break;
        case 2:
            cov <<  0.10, 0.01, 0.02,
                    0.01, 0.20, 0.07,
                    0.02, 0.07, 0.30;
            break;
        default:
            cov <<  0.1, 0.002, 0.003, 
                    0.002, 0.2, 0.006,
                    0.003, 0.006, 0.3;
            cov *= static_cast<double>(i);
            break;
    }
    return cov;
}

Eigen::Matrix<double, 6, 6> test_covariance_6x6(const unsigned int i) {
    Eigen::Matrix<double, 6, 6> cov;
    switch (i) {
        case 0:
            cov <<  0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.025, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.025, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.025;
            break;
        case 1:
            cov <<  0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.3, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.4, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.6;
            break;
        case 2:
            cov <<  0.10, 0.01, 0.02, 0.03, 0.04, 0.05,
                    0.01, 0.20, 0.06, 0.07, 0.08, 0.09,
                    0.02, 0.06, 0.30, 0.011, 0.012, 0.013,
                    0.03, 0.07, 0.011, 0.40, 0.014, 0.015,
                    0.04, 0.08, 0.012, 0.014, 0.50, 0.016,
                    0.05, 0.09, 0.013, 0.015, 0.016, 0.60;
            break;
        default:
            cov <<  0.001, 0.002, 0.003, 0.004, 0.005, 0.006,
                    0.002, 0.007, 0.008, 0.009, 0.010, 0.011,
                    0.003, 0.008, 0.012, 0.013, 0.014, 0.015,
                    0.004, 0.009, 0.013, 0.016, 0.017, 0.018,
                    0.005, 0.010, 0.014, 0.017, 0.019, 0.020,
                    0.006, 0.011, 0.015, 0.018, 0.020, 0.021;
            cov *= static_cast<double>(i);
            break;
    }
    return cov;
}

eigen_ros::Imu test_imu(const unsigned int i) {
    return eigen_ros::Imu{test_quaternion(i), test_vector3(i), test_vector3(i + 1), test_covariance_3x3(i),
            test_covariance_3x3(i + 1), test_covariance_3x3(i + 2), test_time(i), test_string(i)};
}

Eigen::Isometry3d test_isometry3(const unsigned int i) {
    return eigen_ext::to_transform(test_vector3(i), test_quaternion(i));
}

eigen_ros::Odometry test_odometry(const unsigned int i) {
    return eigen_ros::Odometry{test_pose(i), test_twist(i), test_time(i), test_string(i), test_string(i + 1)};
}

eigen_ros::PoseStamped test_pose_stamped(const unsigned int i) {
    return eigen_ros::PoseStamped{test_pose(i), test_time(i)};
}

eigen_ros::Pose test_pose(const unsigned int i) {
    return eigen_ros::Pose{test_vector3(i), test_quaternion(i), test_covariance_6x6(i)};
}

Eigen::Quaterniond test_quaternion(const unsigned int i) {
    return Eigen::Quaterniond{0.7 - 0.03 * i, -1.0 / (i + 1) + 0.123, -0.2 - 0.05 * i, -0.62 + i * 0.07}.normalized();
}

std::string test_string(const unsigned int i) {
    return "test_frame_" + std::to_string(i);
}

ros::Time test_time(const unsigned int i) {
    return ros::Time(1653281971 + i, 123456789 + i);
}

eigen_ros::Twist test_twist(const unsigned int i) {
    return eigen_ros::Twist{test_vector3(i), test_vector3(i + 1), test_covariance_6x6(i)};
}

Eigen::Vector3d test_vector3(const unsigned int i) {
    return Eigen::Vector3d{-0.8 - 3.0 * i, 5.43 + 1.234 * i, -31.999228 + 17.84 * i};
}

void pretest_check_covariance_6x6(const unsigned int i) {
    EXPECT_TRUE(test_covariance_6x6(i).isApprox(test_covariance_6x6(i)));
}

void pretest_check_quaternion(const unsigned int i) {
    EXPECT_TRUE(test_quaternion(i).isApprox(test_quaternion(i)));
}

void pretest_check_vector3(const unsigned int i) {
    EXPECT_TRUE(test_vector3(i).isApprox(test_vector3(i)));
}

TEST(pretest_100, covariance_6x6) {
    for (unsigned int i = 0; i < 100; ++i) {
        pretest_check_covariance_6x6(i);
    }
}

TEST(pretest_100, quaternion) {
    for (unsigned int i = 0; i < 100; ++i) {
        pretest_check_quaternion(i);
    }
}

TEST(pretest_100, vector3) {
    for (unsigned int i = 0; i < 100; ++i) {
        pretest_check_vector3(i);
    }
}
