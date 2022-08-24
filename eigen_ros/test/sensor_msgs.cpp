#include "eigen_ros/sensor_msgs.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "eigen_ros/eigen_ros.hpp"
#include "test/test_instances.hpp"

TEST(imu, to_from_ros) {
    test_to_from<sensor_msgs::Imu>(eigen_ros::Imu{});
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<sensor_msgs::Imu>(test_imu(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "sensor_msgs");
    return RUN_ALL_TESTS();
}
