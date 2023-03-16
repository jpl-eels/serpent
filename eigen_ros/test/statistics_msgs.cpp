#include "eigen_ros/statistics_msgs.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "eigen_ros/eigen_ros.hpp"
#include "test/test_instances.hpp"

TEST(covariance, to_from_ros) {
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<statistics_msgs::Covariance>(test_dynamic_covariance_3x3<double>(i));
    }
}

TEST(covariance32, to_from_ros) {
    for (unsigned int i = 0; i < 100; ++i) {
        test_to_from<statistics_msgs::CovarianceFloat32>(test_dynamic_covariance_3x3<float>(i));
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "statistics_msgs");
    return RUN_ALL_TESTS();
}
