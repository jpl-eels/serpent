#include "test/test_instances.hpp"
#include "eigen_ros/pose.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(pose, equality) {
    EXPECT_EQ(eigen_ros::Pose{}, eigen_ros::Pose{});
    for (unsigned int i = 0; i < 100; ++i) {
        EXPECT_EQ(test_pose(i), test_pose(i));
    }
}

TEST(pose, to_transform) {
    Eigen::Isometry3d transform = to_transform(eigen_ros::Pose{});
    EXPECT_TRUE(transform.matrix().isApprox(Eigen::Matrix4d::Identity()));
    for (unsigned int i = 0; i < 100; ++i) {
        eigen_ros::Pose pose = test_pose(i);
        Eigen::Isometry3d transform = to_transform(pose);
        Eigen::Vector3d translation = transform.translation();
        Eigen::Quaterniond rotation{transform.rotation()};
        EXPECT_TRUE(pose.position.isApprox(translation));
        // We convert the quaternion to a rotation matrix first to avoid the q == -q issue
        EXPECT_TRUE(pose.orientation.toRotationMatrix().isApprox(rotation.toRotationMatrix()));
    }
}

TEST(pose_stamped, equality) {
    EXPECT_EQ(eigen_ros::PoseStamped{}, eigen_ros::PoseStamped{});
    for (unsigned int i = 0; i < 100; ++i) {
        EXPECT_EQ(test_pose_stamped(i), test_pose_stamped(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "pose");
    return RUN_ALL_TESTS();
}
