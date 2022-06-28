#include "navigation_tools/integrate_imu_graph_test.hpp"
#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>
#include <nav_msgs/Odometry.h>

IntegrateImu::IntegrateImu():
    nh("~"), integration_timestamp(ros::Time()), initial_state(gtsam::NavState())
{
    // Publishers
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    path_publisher = nh.advertise<nav_msgs::Path>("path", 1);

    // Subscribers
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("input", 100, &IntegrateImu::integrate, this);

    // Extrinsics
    imu_to_body_ext = Eigen::Quaterniond(nh.param<double>("imu_to_body/w", 1.0),
            nh.param<double>("imu_to_body/x", 0.0), nh.param<double>("imu_to_body/y", 0.0),
            nh.param<double>("imu_to_body/z", 0.0));
    if (imu_to_body_ext.norm() < 0.99 || imu_to_body_ext.norm() > 1.01) {
        ROS_WARN_STREAM("IMU to body extrinsic was not normalised (" << imu_to_body_ext.norm() << "). It will be "
                "normalised.");
    }
    imu_to_body_ext.normalize();
    body_to_imu_ext = imu_to_body_ext.inverse();

    // Integration parameters
    preintegration_params = gtsam::PreintegrationCombinedParams::MakeSharedU(nh.param<double>("gravity", 9.81));
    preintegration_params->setAccelerometerCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/accelerometer", 1.0e-3), 2.0));
    preintegration_params->setGyroscopeCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/gyroscope", 1.0e-3), 2.0));
    preintegration_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/integration", 1.0e-3), 2.0));
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
            std::pow(nh.param<double>("imu_noise/integration_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasAccCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/gyroscope_bias", 1.0e-3), 2.0));
    preintegration_params->print();

    // IMU integrator
    integrator = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_bias);
    integrator_TEST = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_bias_TEST);

    // Optimiser TEST
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
    isam2_params.print();
    optimiser_TEST = std::make_unique<gtsam::ISAM2>(isam2_params);

    // Path
    path.header.frame_id = "map";
}

void IntegrateImu::integrate(const sensor_msgs::Imu::ConstPtr& msg) {
    // Prepare data
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    imu.change_frame(imu_to_body_ext, body_to_imu_ext);
    ROS_WARN_ONCE("IMU msg noise is ignored. Config noise parameters used instead.");

    if (integration_timestamp != ros::Time()) {
        // Integrate
        const double dt = (imu.timestamp - integration_timestamp).toSec();
        if (dt == 0.0) {
            throw std::runtime_error("Integration error: dt == 0.0");
        }
        integrator->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
        integrator_TEST->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);

        // Predict
        const auto state = integrator->predict(initial_state, imu_bias);
        const auto& pose = state.pose();
        const auto& vel = state.velocity();

        // Optimiser TEST
        static int counter = 0;
        if (++counter % 10 == 0) {
            std::cerr << "timestamp:" << imu.timestamp << std::endl;
            state_TEST = integrator_TEST->predict(state_TEST, imu_bias_TEST);
            std::cerr << "Preintegrated Meas Cov:\n" << integrator_TEST->preintMeasCov() << std::endl;
            std::cerr << "Preintegrated Meas Cov Diag:\n" << integrator_TEST->preintMeasCov().diagonal() << std::endl;
            new_factors.emplace_shared<gtsam::CombinedImuFactor>(X(key - 1), V(key - 1), X(key), V(key), B(key - 1),
                    B(key), *integrator_TEST);
            new_values.insert(X(key), state_TEST.pose());
            new_values.insert(V(key), state_TEST.velocity());
            new_values.insert(B(key), imu_bias_TEST);
            auto result = optimiser_TEST->update(new_factors, new_values);
            std::cerr << "optimiser before/after errors: " << *result.errorBefore << ", " << *result.errorAfter << "\n";
            integrator_TEST->resetIntegrationAndSetBias(imu_bias_TEST);
            new_factors = gtsam::NonlinearFactorGraph();
            new_values.clear();
            gtsam::Pose3 pose_TEST = optimiser_TEST->calculateEstimate<gtsam::Pose3>(X(key));
            Eigen::Quaterniond q = pose_TEST.rotation().toQuaternion();
            Eigen::Matrix<double, 6, 6> pose_cov_TEST = optimiser_TEST->marginalCovariance(X(key));
            std::cerr << "pose_TEST:\n" << pose_TEST.translation().vector() << "\n" << q.w() << ", " << q.x() << ", "
                    << q.y() << ", " << q.z() << "\n";
            std::cerr << "pose_cov_TEST:\n" << pose_cov_TEST << "\n";
            std::cerr << "pose_cov_TEST diag:\n" << pose_cov_TEST.diagonal() << "\n";
            gtsam::Velocity3 vel_TEST = optimiser_TEST->calculateEstimate<gtsam::Velocity3>(V(key));
            Eigen::Matrix3d vel_cov_TEST = optimiser_TEST->marginalCovariance(V(key));
            std::cerr << "vel_TEST:\n" << vel_TEST << "\n";
            std::cerr << "vel_cov_TEST:\n" << vel_cov_TEST << "\n";
            std::cerr << "vel_cov_TEST diag:\n" << vel_cov_TEST.diagonal() << "\n";
            gtsam::imuBias::ConstantBias bias_TEST = optimiser_TEST->calculateEstimate<gtsam::imuBias::ConstantBias>(
                    B(key));
            Eigen::Matrix<double, 6, 6> bias_cov_TEST = optimiser_TEST->marginalCovariance(B(key));
            std::cerr << "bias_TEST:\n" << bias_TEST << "\n";
            std::cerr << "bias_cov_TEST:\n" << bias_cov_TEST << "\n";
            std::cerr << "bias_cov_TEST diag:\n" << bias_cov_TEST.diagonal() << "\n";
            ++key;
        }

        // Covariance
        const Eigen::Matrix<double, 15, 15> integration_covariance = integrator->preintMeasCov();
        Eigen::Matrix<double, 6, 6> pose_covariance = integration_covariance.block<6, 6>(0, 0);
        const Eigen::Matrix3d linear_velocity_covariance = integration_covariance.block<3, 3>(6, 6);
        Eigen::Matrix<double, 6, 6> velocity_covariance;
        velocity_covariance << imu.angular_velocity_covariance, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                linear_velocity_covariance;

        // Publish odometry
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = "map";
        odometry->child_frame_id = "body_i-1";
        eigen_ros::to_ros(odometry->pose.pose.position, pose.translation());
        eigen_ros::to_ros(odometry->pose.pose.orientation, pose.rotation().toQuaternion());
        eigen_ros::to_ros(odometry->pose.covariance, eigen_ext::reorder_covariance(pose_covariance, 3));
        eigen_ros::to_ros(odometry->twist.twist.linear, vel);
        eigen_ros::to_ros(odometry->twist.twist.angular, imu.angular_velocity);
        eigen_ros::to_ros(odometry->twist.covariance, eigen_ext::reorder_covariance(velocity_covariance, 3));
        odometry_publisher.publish(odometry);

        // Publish path
        path.header.stamp = imu.timestamp;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = imu.timestamp;
        pose_stamped.header.frame_id = "body_i-1";
        pose_stamped.pose = odometry->pose.pose;
        path.poses.emplace_back(pose_stamped);
        path_publisher.publish(path);
    } else {
        const Eigen::Quaterniond orientation = imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0)) ?
                Eigen::Quaterniond::Identity() : imu.orientation;
        const gtsam::Point3 position{nh.param<double>("pose/position/x", 0.0),
                nh.param<double>("pose/position/y", 0.0), nh.param<double>("pose/position/z", 0.0)};
        const gtsam::Velocity3 linear_velocity{nh.param<double>("velocity/linear/x", 0.0),
                nh.param<double>("velocity/linear/y", 0.0), nh.param<double>("velocity/linear/z", 0.0)};
        initial_state = gtsam::NavState{gtsam::Rot3(orientation), position, linear_velocity};

        state_TEST = initial_state;
        const gtsam::Pose3 initial_pose = gtsam::Pose3{gtsam::Rot3(orientation), position};
        const gtsam::SharedNoiseModel pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
                0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), initial_pose, pose_noise);
        const gtsam::SharedNoiseModel vel_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
                0.001, 0.001, 0.001).finished());
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Velocity3>>(V(0), linear_velocity, vel_noise);
        const gtsam::SharedNoiseModel bias_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
                0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001).finished());
        new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), imu_bias_TEST, bias_noise);
        new_values.insert(X(0), initial_pose);
        new_values.insert(V(0), linear_velocity);
        new_values.insert(B(0), imu_bias_TEST);
    }
    integration_timestamp = imu.timestamp;
}
