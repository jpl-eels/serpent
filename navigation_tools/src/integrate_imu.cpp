#include "navigation_tools/integrate_imu.hpp"
#include <nav_msgs/Odometry.h>

IntegrateImu::IntegrateImu():
    nh("~"), integration_timestamp(ros::Time()), state(gtsam::NavState())
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

    // IMU integrator
    integrator = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_bias);

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

        // Predict
        state = integrator->predict(state, imu_bias);
        auto pose = state.pose();
        auto vel = state.velocity();

        // Reset
        integrator->resetIntegrationAndSetBias(imu_bias);

        // Publish odometry
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = "map";
        odometry->child_frame_id = "body_i-1";
        eigen_ros::to_ros(odometry->pose.pose.position, pose.translation());
        eigen_ros::to_ros(odometry->pose.pose.orientation, pose.rotation().toQuaternion());
        // eigen_ros::to_ros(odometry->pose.covariance, covariance);
        ROS_WARN_ONCE("Odometry covariance currently not set.");
        eigen_ros::to_ros(odometry->twist.twist.linear, vel);
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
        const Eigen::Vector3d position{nh.param<double>("pose/position/x", 0.0),
                nh.param<double>("pose/position/y", 0.0), nh.param<double>("pose/position/z", 0.0)};
        state = gtsam::NavState{gtsam::Rot3(orientation), position, gtsam::Velocity3(0.0, 0.0, 0.0)};
    }
    integration_timestamp = imu.timestamp;
}
