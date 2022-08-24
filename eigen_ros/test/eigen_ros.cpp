#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test/test_instances.hpp"

TEST(matrix, to_from_ros_3x3) {
    Eigen::Matrix3d m;
    m << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
    test_to_from<boost::array<double, 3 * 3>>(m);
}

TEST(matrix, to_from_ros_6x6) {
    Eigen::Matrix<double, 6, 6> m;
    m <<  1.0,  2.0,   3.0, 4.0,  5.0,  6.0,
          7.0,  8.0,  9.0, 10.0, 11.0, 12.0,
         13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
         19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
         25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
         31.0, 32.0, 33.0, 34.0, 35.0, 36.0;
    test_to_from<boost::array<double, 6 * 6>>(m);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "eigen_ros");
    return RUN_ALL_TESTS();
}
