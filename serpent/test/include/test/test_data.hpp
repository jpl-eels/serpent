#ifndef SERPENT_TEST_TEST_DATA_HPP
#define SERPENT_TEST_TEST_DATA_HPP

#include <Eigen/Geometry>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <ros/time.h>

Eigen::Matrix<double, 6, 6> test_imu_bias_covariance(const unsigned int i);

gtsam::SharedNoiseModel test_imu_bias_noise_model(const unsigned int i);

gtsam::imuBias::ConstantBias test_imu_bias(const unsigned int i);

gtsam::ISAM2Params test_isam2_params();

gtsam::NavState test_navstate(const unsigned int i);

Eigen::Matrix<double, 6, 6> test_pose_covariance(const unsigned int i);

gtsam::SharedNoiseModel test_pose_noise_model(const unsigned int i);

gtsam::Pose3 test_pose(const unsigned int i);

gtsam::Point3 test_point(const unsigned int i);

gtsam::PreintegratedCombinedMeasurements test_preintegrated_measurements();

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> test_preintegrated_params();

Eigen::Quaterniond test_quaternion(const unsigned int i);

gtsam::Rot3 test_rotation(const unsigned int i);

ros::Time test_timestamp(const unsigned int i);

Eigen::Matrix3d test_velocity_covariance(const unsigned int i);

gtsam::SharedNoiseModel test_velocity_noise_model(const unsigned int i);

gtsam::Velocity3 test_velocity(const unsigned int i);

#endif
