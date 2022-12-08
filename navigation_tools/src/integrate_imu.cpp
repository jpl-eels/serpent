#include "navigation_tools/integrate_imu.hpp"

#include <nav_msgs/Odometry.h>

#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>

IntegrateImu::IntegrateImu()
    : nh("~"),
      integration_timestamp(ros::Time()),
      initial_state(gtsam::NavState()) {
    // Publishers
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    path_publisher = nh.advertise<nav_msgs::Path>("path", 1);

    // Subscribers
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("input", 100, &IntegrateImu::integrate, this);

    // Integration parameters
    preintegration_params = gtsam::PreintegrationCombinedParams::MakeSharedD(nh.param<double>("gravity", 9.81));
    preintegration_params->setAccelerometerCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer", 1.0e-3), 2.0));
    preintegration_params->setGyroscopeCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope", 1.0e-3), 2.0));
    preintegration_params->setIntegrationCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/integration", 1.0e-3), 2.0));
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
                                              std::pow(nh.param<double>("imu/noise/integration_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasAccCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope_bias", 1.0e-3), 2.0));
    // pose of the sensor in the body frame
    const gtsam::Pose3 body_to_imu = eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame("imu"));
    preintegration_params->setBodyPSensor(body_to_imu);

    // IMU integrator
    integrator = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_bias);

    // Path
    path.header.frame_id = "map";
}

void IntegrateImu::integrate(const sensor_msgs::Imu::ConstPtr& msg) {
    // Prepare data
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);

    if (integration_timestamp != ros::Time()) {
        // Integrate
        const double dt = (imu.timestamp - integration_timestamp).toSec();
        if (dt == 0.0) {
            throw std::runtime_error("Integration error: dt == 0.0");
        }
        integrator->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);

        // Predict
        const auto state = integrator->predict(initial_state, imu_bias);
        const auto& pose = state.pose();
        const auto& vel = state.velocity();

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
        const Eigen::Quaterniond body_orientation =
                imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0))
                        ? Eigen::Quaterniond::Identity()
                        : Eigen::Quaterniond{(imu.orientation * body_frames.frame_to_body("imu")).rotation()};
        const gtsam::Point3 position{nh.param<double>("pose/position/x", 0.0), nh.param<double>("pose/position/y", 0.0),
                nh.param<double>("pose/position/z", 0.0)};
        const gtsam::Velocity3 linear_velocity{nh.param<double>("velocity/linear/x", 0.0),
                nh.param<double>("velocity/linear/y", 0.0), nh.param<double>("velocity/linear/z", 0.0)};
        initial_state = gtsam::NavState{gtsam::Rot3{body_orientation}, position, linear_velocity};
    }
    integration_timestamp = imu.timestamp;
}
