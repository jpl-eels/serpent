#include "eigen_ros/imu.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "eigen_ext/matrix.hpp"
#include "test/test_instances.hpp"

TEST(imu, interpolate_trivial_0) {
    eigen_ros::Imu imu0 = test_imu(0);
    eigen_ros::Imu imu1 = test_imu(1);
    eigen_ros::Imu imu_interp = eigen_ros::interpolate(imu0, imu1, imu0.timestamp);
    EXPECT_EQ(imu0, imu_interp);
}

TEST(imu, interpolate_trivial_1) {
    eigen_ros::Imu imu0 = test_imu(0);
    eigen_ros::Imu imu1 = test_imu(1);
    eigen_ros::Imu imu_interp = eigen_ros::interpolate(imu0, imu1, imu1.timestamp);
    EXPECT_EQ(imu1, imu_interp);
}

TEST(imu, interpolate_0) {
    eigen_ros::Imu imu0 = test_imu(0);
    eigen_ros::Imu imu1 = test_imu(1);
    const double interp_true{0.3};
    ros::Time interp_timestamp = imu0.timestamp + (imu1.timestamp - imu0.timestamp) * interp_true;
    const double interp = (interp_timestamp - imu0.timestamp).toSec() / (imu1.timestamp - imu0.timestamp).toSec();
    eigen_ros::Imu imu_interp = eigen_ros::interpolate(imu0, imu1, interp_timestamp);
    EXPECT_TRUE(imu_interp.orientation.isApprox(imu0.orientation.slerp(interp, imu1.orientation)));
    Eigen::Quaterniond slerp = imu0.orientation.slerp(interp, imu1.orientation);
    EXPECT_TRUE(imu_interp.angular_velocity.isApprox(
            eigen_ext::linear_interpolate(imu0.angular_velocity, imu1.angular_velocity, interp)));
    EXPECT_TRUE(imu_interp.linear_acceleration.isApprox(
            eigen_ext::linear_interpolate(imu0.linear_acceleration, imu1.linear_acceleration, interp)));
    EXPECT_TRUE(imu_interp.orientation_covariance.isApprox(imu0.orientation_covariance));
    EXPECT_TRUE(imu_interp.angular_velocity_covariance.isApprox(imu0.angular_velocity_covariance));
    EXPECT_TRUE(imu_interp.linear_acceleration_covariance.isApprox(imu0.linear_acceleration_covariance));
    EXPECT_EQ(imu_interp.frame, imu0.frame);
    EXPECT_EQ(imu_interp.timestamp, interp_timestamp);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "imu");
    return RUN_ALL_TESTS();
}
