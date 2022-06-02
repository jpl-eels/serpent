#include "test/test_instances.hpp"
#include "eigen_ros/odometry.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(odometry, equality) {
    EXPECT_EQ(eigen_ros::Odometry{}, eigen_ros::Odometry{});
    for (unsigned int i = 0; i < 100; ++i) {
        EXPECT_EQ(test_odometry(i), test_odometry(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "odometry");
    return RUN_ALL_TESTS();
}
