#include "test/test_instances.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(point, to_from_ros) {
    test_to_from<geometry_msgs::Point>(Eigen::Vector3d{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<geometry_msgs::Point>(test_vector3(i));
    }
}

TEST(pose_with_covariance, to_from_ros) {
    test_to_from<geometry_msgs::PoseWithCovariance>(eigen_ros::Pose{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<geometry_msgs::PoseWithCovariance>(test_pose(i));
    }
}

TEST(pose_with_covariance_stamped, to_from_ros) {
    test_to_from<geometry_msgs::PoseWithCovarianceStamped>(eigen_ros::PoseStamped{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<geometry_msgs::PoseWithCovarianceStamped>(test_pose_stamped(i));
    }
}

TEST(transform, to_from_ros) {
    test_to_from_approx<geometry_msgs::Transform>(Eigen::Isometry3d::Identity());
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from_approx<geometry_msgs::Transform>(test_isometry3(i));
    }
}

TEST(quaternion, to_from_ros) {
    test_to_from_approx<geometry_msgs::Quaternion>(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from_approx<geometry_msgs::Quaternion>(test_quaternion(i));
    }
}

TEST(twist_with_covariance, to_from_ros) {
    test_to_from<geometry_msgs::TwistWithCovariance>(eigen_ros::Twist{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<geometry_msgs::TwistWithCovariance>(test_twist(i));
    }
}

TEST(vector3, to_from_ros) {
    test_to_from_approx<geometry_msgs::Vector3, Eigen::Vector3d>(Eigen::Vector3d::Zero());
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from_approx<geometry_msgs::Vector3>(test_vector3(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "geometry_msgs");
    return RUN_ALL_TESTS();
}
