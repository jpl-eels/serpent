#include <gtest/gtest.h>
#include <ros/time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "eigen_ros/eigen_ros.hpp"
#include "eigen_ros/geometry_msgs.hpp"
#include "eigen_ros/imu.hpp"
#include "eigen_ros/nav_msgs.hpp"
#include "eigen_ros/odometry.hpp"
#include "eigen_ros/pose.hpp"
#include "eigen_ros/sensor_msgs.hpp"
#include "eigen_ros/twist.hpp"

Eigen::Matrix3d test_covariance_3x3(const unsigned int i);

Eigen::Matrix<double, 6, 6> test_covariance_6x6(const unsigned int i);

eigen_ros::Imu test_imu(const unsigned int i);

Eigen::Isometry3d test_isometry3(const unsigned int i);

eigen_ros::Odometry test_odometry(const unsigned int i);

eigen_ros::PoseStamped test_pose_stamped(const unsigned int i);

eigen_ros::Pose test_pose(const unsigned int i);

Eigen::Quaterniond test_quaternion(const unsigned int i);

std::string test_string(const unsigned int i);

ros::Time test_time(const unsigned int i);

eigen_ros::Twist test_twist(const unsigned int i);

Eigen::Vector3d test_vector3(const unsigned int i);

void pretest_check_covariance_6x6(const unsigned int i);

void pretest_check_quaternion(const unsigned int i);

void pretest_check_vector3(const unsigned int i);

template<typename RosType, typename Type>
void test_to_from(const Type& in) {
    RosType msg = eigen_ros::to_ros<RosType>(in);
    Type out = eigen_ros::from_ros<Type>(msg);
    EXPECT_EQ(in, out);
}

template<typename RosType, typename Type>
void test_to_from_approx(const Type& in) {
    RosType msg = eigen_ros::to_ros<RosType>(in);
    Type out = eigen_ros::from_ros<Type>(msg);
    EXPECT_TRUE(in.isApprox(out));
}
