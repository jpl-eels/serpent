#include "eigen_ros/pose.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test/test_instances.hpp"

TEST(pose, equality) {
    EXPECT_EQ(eigen_ros::Pose{}, eigen_ros::Pose{});
    for (unsigned int i = 0; i < 100; ++i) {
        EXPECT_EQ(test_pose(i), test_pose(i));
    }
}

TEST(pose, apply_transform) {
    // NOTE: does not test covariance combination
    eigen_ros::Pose identity;
    for (unsigned int i = 0; i < 100; ++i) {
        eigen_ros::Pose transform = test_pose(i);
        eigen_ros::Pose transformed = apply_transform(identity, transform);
        // TODO FIX: test covariances
        EXPECT_TRUE(eigen_ros::to_transform(transform).isApprox(eigen_ros::to_transform(transformed)));
        eigen_ros::Pose rotation{Eigen::Vector3d::Zero(), test_quaternion(i)};
        eigen_ros::Pose translation{test_vector3(i)};
        Eigen::Isometry3d pose_eigen = eigen_ros::to_transform(rotation) * to_transform(translation);
        eigen_ros::Pose pose = eigen_ros::apply_transform(rotation, translation);
        EXPECT_TRUE(eigen_ros::to_transform(pose).isApprox(pose_eigen));
    }
}

TEST(pose, to_transform) {
    Eigen::Isometry3d transform = to_transform(eigen_ros::Pose{});
    EXPECT_TRUE(transform.matrix().isApprox(Eigen::Matrix4d::Identity()));
    for (unsigned int i = 0; i < 100; ++i) {
        const eigen_ros::Pose pose = test_pose(i);
        const Eigen::Isometry3d transform = to_transform(pose);
        const Eigen::Vector3d translation = transform.translation();
        const Eigen::Quaterniond rotation{transform.rotation()};
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
