#include "eigen_ros/odometry.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test/test_instances.hpp"

TEST(odometry, equality) {
    EXPECT_EQ(eigen_ros::Odometry{}, eigen_ros::Odometry{});
    for (unsigned int i = 0; i < 100; ++i) {
        EXPECT_EQ(test_odometry(i), test_odometry(i));
    }
}

TEST(odometry, apply_transformation) {
    eigen_ros::Odometry identity;
    for (unsigned int i = 0; i < 100; ++i) {
        eigen_ros::PoseStamped transform = test_pose_stamped(i);
        eigen_ros::Odometry transformed = eigen_ros::apply_transform(identity, transform);
        EXPECT_TRUE(eigen_ros::to_transform(transformed.pose).isApprox(eigen_ros::to_transform(transform.data)));
        // TODO FIX: test covariances
        EXPECT_EQ(transformed.twist, identity.twist);
        EXPECT_EQ(transformed.timestamp, transform.timestamp);
        EXPECT_EQ(transformed.frame, identity.frame);
        EXPECT_EQ(transformed.child_frame, identity.child_frame);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "odometry");
    return RUN_ALL_TESTS();
}
