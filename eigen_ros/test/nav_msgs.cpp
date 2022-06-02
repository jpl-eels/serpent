#include "test/test_instances.hpp"
#include "eigen_ros/nav_msgs.hpp"
#include "eigen_ros/eigen_ros.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(odometry, to_from_ros) {
    test_to_from<nav_msgs::Odometry>(eigen_ros::Odometry{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<nav_msgs::Odometry>(test_odometry(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "nav_msgs");
    return RUN_ALL_TESTS();
}
