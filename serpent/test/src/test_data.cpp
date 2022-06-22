#include "test/test_data.hpp"

Eigen::Matrix<double, 6, 6> test_imu_bias_covariance(const unsigned int i) {
    Eigen::Matrix<double, 6, 6> cov;
    const double i_ = static_cast<double>(i);
    const double accel_noise = (i_ + 1.0) * 1.0e-5;
    const double gyro_noise = (i_ + 1.0) * 1.0e-4;
    cov << Eigen::Matrix3d::Identity() * std::pow(accel_noise, 2.0), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() * std::pow(gyro_noise, 2.0);
    return cov;
}

gtsam::SharedNoiseModel test_imu_bias_noise_model(const unsigned int i) {
    return gtsam::noiseModel::Gaussian::Covariance(test_imu_bias_covariance(i));
}

gtsam::imuBias::ConstantBias test_imu_bias(const unsigned int i) {
    const double i_ = static_cast<double>(i);
    return gtsam::imuBias::ConstantBias{gtsam::Vector3{0.0 + 0.01 * i_, 0.0 - 0.011 * i_, 0.0 + 0.012 * i_},
            gtsam::Vector3{0.0 - 0.1 * i_, 0.0 + 0.2 * i_, 0.0 - 0.15 * i_}};
}

gtsam::ISAM2Params test_isam2_params() {
    gtsam::ISAM2Params isam2_params;
    gtsam::ISAM2GaussNewtonParams optimization_params = gtsam::ISAM2GaussNewtonParams();
    optimization_params.setWildfireThreshold(1.0e-3);
    isam2_params.setOptimizationParams(optimization_params);
    isam2_params.setRelinearizeThreshold(0.1);
    isam2_params.setRelinearizeSkip(10);
    isam2_params.setEnableRelinearization(true);
    isam2_params.setEvaluateNonlinearError(true);
    isam2_params.setFactorization("CHOLESKY");
    isam2_params.setCacheLinearizedFactors(true);
    return isam2_params;
}

gtsam::NavState test_navstate(const unsigned int i) {
    return gtsam::NavState(test_pose(i), test_velocity(i));
}

gtsam::Pose3 test_pose(const unsigned int i) {
    return gtsam::Pose3{test_rotation(i), test_point(i)};
}

Eigen::Matrix<double, 6, 6> test_pose_covariance(const unsigned int i) {
    Eigen::Matrix<double, 6, 6> cov;
    const double i_ = static_cast<double>(i);
    const double rotation_noise = (i_ + 1.0) * 1.0e-2;
    const double position_noise = (i_ + 1.0) * 1.0e-1;
    cov <<  Eigen::Matrix3d::Identity() * std::pow(rotation_noise, 2.0), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() * std::pow(position_noise, 2.0);
    return cov;
}

gtsam::SharedNoiseModel test_pose_noise_model(const unsigned int i) {
    return gtsam::noiseModel::Gaussian::Covariance(test_pose_covariance(i));
}

gtsam::Point3 test_point(const unsigned int i) {
    const double i_ = static_cast<double>(i);
    return gtsam::Point3{-9.7 + 0.789 * i_, 0.0 + 1.89 * i_, 16.32134 - 5.9 * i_};
}

gtsam::PreintegratedCombinedMeasurements test_preintegrated_measurements() {
    return gtsam::PreintegratedCombinedMeasurements{test_preintegrated_params(), gtsam::imuBias::ConstantBias{}};
}

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> test_preintegrated_params() {
    auto preintegrated_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.81);
    preintegrated_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setBiasAccCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setBiasOmegaCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setAccelerometerCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setGyroscopeCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    preintegrated_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() * std::pow(1.0e-3, 2.0));
    return preintegrated_params;
}

Eigen::Quaterniond test_quaternion(const unsigned int i) {
    const double i_ = static_cast<double>(i);
    return Eigen::Quaterniond{0.25 - 0.05 * i_, -0.3 + 0.1 * i_, 0.71 - 0.2 * i_, -0.6 + 0.1 * i_}.normalized();
}

gtsam::Rot3 test_rotation(const unsigned int i) {
    return gtsam::Rot3{test_quaternion(i)};
}

ros::Time test_timestamp(const unsigned int i) {
    return ros::Time(static_cast<double>(i));
}

Eigen::Matrix3d test_velocity_covariance(const unsigned int i) {
    const double i_ = static_cast<double>(i);
    return Eigen::Matrix3d::Identity() * std::pow((i_ + 1.0) * 1.0e-1, 2.0);
}

gtsam::SharedNoiseModel test_velocity_noise_model(const unsigned int i) {
    return gtsam::noiseModel::Gaussian::Covariance(test_velocity_covariance(i));
}

gtsam::Velocity3 test_velocity(const unsigned int i) {
    const double i_ = static_cast<double>(i);
    return gtsam::Velocity3{0.0 - 1.89 * i_, 2.4 + 0.789 * i_, -16.122 + 4.1 * i_};
}
