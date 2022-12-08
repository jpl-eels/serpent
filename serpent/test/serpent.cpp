#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <ros/ros.h>

#include <eigen_ros/eigen_ros.hpp>

#include "serpent/ImuBiases.h"
#include "serpent/utilities.hpp"
#include "test/read_data.hpp"
#include "test/test_utils.hpp"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

TEST(serpent, isam2_optimise) {
    // Noise characteristics
    const double position_noise{1.0e-3};
    const double rotation_noise{1.0e-3};
    const double vel_noise{1.0e-3};
    const double accel_noise{1.0e-3};
    const double gyro_noise{1.0e-3};
    const double integration_noise{1.0e-3};
    const double overwrite_accelerometer_noise{1.0e-3};
    const double overwrite_gyroscope_noise{1.0e-3};

    // Noise models
    const auto prior_pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << rotation_noise,
            rotation_noise, rotation_noise, position_noise, position_noise, position_noise)
                                                                              .finished());
    const auto prior_vel_noise =
            gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << vel_noise, vel_noise, vel_noise).finished());
    const auto prior_bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << accel_noise, accel_noise, accel_noise, gyro_noise, gyro_noise, gyro_noise).finished());
    const auto between_pose_noise = prior_pose_noise;
    const auto between_bias_noise = prior_pose_noise;
    const Eigen::Matrix3d integration_covariance = Eigen::Matrix3d::Identity() * std::pow(integration_noise, 2.0);
    const Eigen::Matrix3d overwrite_accelerometer_covariance =
            Eigen::Matrix3d::Identity() * std::pow(overwrite_accelerometer_noise, 2.0);
    const Eigen::Matrix3d overwrite_gyroscope_covariance =
            Eigen::Matrix3d::Identity() * std::pow(overwrite_gyroscope_noise, 2.0);

    // Data structures setup
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    gtsam::ISAM2 isam2;
    auto zero_bias = gtsam::imuBias::ConstantBias(gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0));
    gtsam::Key key{0};

    // IMU measurements, set preintegration noise
    auto imu_measurements = from_ros(read_imu(std::string(DATA_DIR) + "/imu_up_10msg.bag", "/imu"));
    EXPECT_EQ(imu_measurements.size(), 10);

    // Priors
    gtsam::NavState prior_state{gtsam::Rot3(1.0, 0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0),
            gtsam::Velocity3(0.0, 0.0, 0.0)};
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), prior_state.pose(), prior_pose_noise);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(key), prior_state.velocity(), prior_vel_noise);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(key), zero_bias, prior_bias_noise);
    new_values.insert(X(key), prior_state.pose());
    new_values.insert(V(key), prior_state.velocity());
    new_values.insert(B(key), zero_bias);
    ++key;

    // Preintegration setup using overwritten covariances
    auto preintegration_params = gtsam::PreintegrationParams::MakeSharedD(9.81);
    preintegration_params->setAccelerometerCovariance(overwrite_accelerometer_covariance);
    EXPECT_GT(preintegration_params->getAccelerometerCovariance().diagonal().minCoeff(), 0.0);
    preintegration_params->setGyroscopeCovariance(overwrite_gyroscope_covariance);
    EXPECT_GT(preintegration_params->getGyroscopeCovariance().diagonal().minCoeff(), 0.0);
    preintegration_params->setIntegrationCovariance(integration_covariance);
    EXPECT_GT(preintegration_params->getIntegrationCovariance().diagonal().minCoeff(), 0.0);
    gtsam::PreintegratedImuMeasurements preintegrated_imu{preintegration_params, zero_bias};

    // Preintegration
    for (std::size_t i = 1; i < imu_measurements.size(); ++i) {
        const auto& previous = imu_measurements.at(i - 1);
        const auto& current = imu_measurements.at(i);
        const double dt = (current.timestamp - previous.timestamp).toSec();
        EXPECT_GT(dt, 0.0);
        preintegrated_imu.integrateMeasurement(current.linear_acceleration, current.angular_velocity, dt);
    }
    gtsam::NavState predicted = preintegrated_imu.predict(prior_state, zero_bias);

    // IMU and Bias Between factors
    new_factors.emplace_shared<gtsam::ImuFactor>(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preintegrated_imu);
    new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(key - 1), B(key), zero_bias,
            between_bias_noise);
    new_values.insert(X(key), predicted.pose());
    new_values.insert(V(key), predicted.velocity());
    new_values.insert(B(key), zero_bias);

    // Generate a guessed between factor close to the IMU with a 5 degree rotation and 2% translation change
    gtsam::Pose3 registration_pose{
            predicted.pose().rotation() * gtsam::Rot3(Eigen::Quaterniond(0.999, 0.044, 0.0, 0.0).normalized()),
            predicted.pose().translation() * 0.98};

    // Registration between factor
    new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(key - 1), X(key), registration_pose);
    new_values.update(X(key), registration_pose);

    // Optimise
    gtsam::ISAM2Result optimisation_result = isam2.update(new_factors, new_values);
    EXPECT_EQ(optimisation_result.getVariablesReeliminated(), 6);
    EXPECT_EQ(optimisation_result.getVariablesRelinearized(), 0);
    EXPECT_EQ(optimisation_result.getCliques(), 6);

    // Get estimate
    gtsam::Values optimised_values = isam2.calculateEstimate();
    EXPECT_EQ(optimised_values.size(), 6);
}

void to_from_ros_check(const gtsam::imuBias::ConstantBias& in) {
    serpent::ImuBiases msg;
    serpent::to_ros(msg, in, ros::Time(0), std::string());
    gtsam::imuBias::ConstantBias out;
    serpent::from_ros(msg, out);
    EXPECT_TRUE(in.vector().isApprox(out.vector()));
}

TEST(imu_bias, to_from_ros) {
    to_from_ros_check(gtsam::imuBias::ConstantBias{});
    to_from_ros_check(gtsam::imuBias::ConstantBias{gtsam::Vector3{0.1, 0.1, 0.1}, gtsam::Vector3{0.2, 0.2, 0.2}});
    to_from_ros_check(gtsam::imuBias::ConstantBias{gtsam::Vector3{0.01, 0.02, 0.03}, gtsam::Vector3{0.04, 0.05, 0.06}});
}
