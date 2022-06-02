#include "serpent/optimisation.hpp"
#include "serpent/utilities.hpp"
#include "serpent/ImuBiases.h"
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/nav_msgs.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

namespace serpent {

Optimisation::Optimisation():
    nh("~")
{
    // Publishers
    imu_biases_publisher = nh.advertise<serpent::ImuBiases>("optimisation/imu_biases", 1);
    imu_transform_publisher = nh.advertise<geometry_msgs::TransformStamped>("optimisation/imu_transform", 1);
    optimised_odometry_publisher = nh.advertise<nav_msgs::Odometry>("optimisation/odometry", 1);
    path_publisher = nh.advertise<nav_msgs::Path>("output/path", 1);
    path_changes_publisher = nh.advertise<nav_msgs::Path>("optimisation/path_changes", 1);

    // Subscribers
    imu_s2s_subscriber = nh.subscribe<serpent::ImuArray>("frontend/imu_s2s", 10, &Optimisation::imu_s2s_callback,
            this);
    initial_odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("frontend/initial_odometry", 1,
            &Optimisation::initial_odometry_callback, this);
    registration_transform_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("registration/transform",
            10, &Optimisation::registration_transform_callback, this);
    
    // Preintegration parameters
    preintegration_params = gtsam::PreintegrationCombinedParams::MakeSharedU(nh.param<double>("gravity", 9.81));
    ROS_WARN_ONCE("DESIGN DECISION: gravity from initialisation procedure?");
    preintegration_params->setIntegrationCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/integration", 1.0e-3), 2.0));
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
            std::pow(nh.param<double>("imu_noise/integration_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasAccCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(Eigen::Matrix3d::Identity() *
            std::pow(nh.param<double>("imu_noise/gyroscope_bias", 1.0e-3), 2.0));
    // Optimiser
    gtsam::ISAM2Params isam2_params;
    const std::string optimization = nh.param<std::string>("isam2/optimization", "GaussNewton");
    if (optimization == "GaussNewton") {
        gtsam::ISAM2GaussNewtonParams optimization_params = gtsam::ISAM2GaussNewtonParams();
        optimization_params.setWildfireThreshold(nh.param<double>("isam2/gauss_newton_params/wildfire_threshold",
                1.0e-3));
        isam2_params.setOptimizationParams(optimization_params);
    } else if (optimization == "Dogleg") {
        gtsam::ISAM2DoglegParams optimization_params = gtsam::ISAM2DoglegParams();
        optimization_params.setInitialDelta(nh.param<double>("isam2/dogleg_params/initial_delta", 1.0));
        optimization_params.setWildfireThreshold(nh.param<double>("isam2/dogleg_params/wildfire_threshold", 1.0e-5));
        optimization_params.setAdaptationMode(nh.param<std::string>("isam2/dogleg_params/trust_region_adaptation_mode",
                "SEARCH_EACH_ITERATION"));
        optimization_params.setVerbose(nh.param<bool>("isam2/dogleg_params/verbose", false));
        isam2_params.setOptimizationParams(optimization_params);
    } else {
        throw std::runtime_error("Unrecognised isam2 optimisation method " + optimization);
    }
    isam2_params.setRelinearizeThreshold(nh.param<double>("isam2/relinearize_threshold", 0.1));
    isam2_params.setRelinearizeSkip(nh.param<int>("isam2/relinearize_skip", 10));
    isam2_params.setEnableRelinearization(nh.param<bool>("isam2/enable_relinearization", true));
    isam2_params.setEvaluateNonlinearError(nh.param<bool>("isam2/evaluate_nonlinear_error", false));
    isam2_params.setFactorization(nh.param<std::string>("isam2/factorization", "CHOLESKY"));
    isam2_params.setCacheLinearizedFactors(nh.param<bool>("isam2/cache_linearized_factors", true));
    optimiser = std::make_unique<gtsam::ISAM2>(isam2_params);
}

void Optimisation::imu_s2s_callback(const serpent::ImuArray::ConstPtr& msg) {
    // Deserialise imu array
    ros::Time last_preint_imu_timestamp = msg->start_timestamp;
    std::deque<eigen_ros::Imu> imu_s2s;
    from_ros(msg->measurements, imu_s2s);
    if (imu_s2s.empty()) {
        throw std::runtime_error("S2S imu array was empty");
    }

    // Create IMU preintegration
    update_preintegration_params(*preintegration_params, imu_s2s.front().linear_acceleration_covariance,
            imu_s2s.front().angular_velocity_covariance);
    gtsam::PreintegratedCombinedMeasurements preintegrated_imu{preintegration_params, imu_bias};

    // Preintegrate IMU measurements over sweep
    for (const auto& imu : imu_s2s) {
        double dt = (imu.timestamp - last_preint_imu_timestamp).toSec();
        preintegrated_imu.integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
        last_preint_imu_timestamp = imu.timestamp;
    }
    preintegrated_imu.integrateMeasurement(imu_s2s.back().linear_acceleration, imu_s2s.back().angular_velocity,
            (msg->end_timestamp - imu_s2s.back().timestamp).toSec());
    
    // Predict change in pose from preintegration
    gtsam::NavState s2s_state = preintegrated_imu.predict(gtsam::NavState(), imu_bias);
    std::lock_guard<std::mutex> guard(graph_mutex);
    gtsam::NavState state = preintegrated_imu.predict(optimised_state, imu_bias);
    ROS_WARN_ONCE("DESIGN DECISION: should the registration module take incremental or world? If world, we can use the"
            " predict function on optimised_state and then in registration module calculate the incremental");
    eigen_ros::Pose s2s_pose{s2s_state.pose().translation(), s2s_state.pose().rotation().toQuaternion()};
    ROS_WARN_ONCE("DESIGN DECISION: change output to odometry to pass velocity & covariances to registration module");

    // Publish imu estimated transform
    auto imu_transform = boost::make_shared<geometry_msgs::TransformStamped>();
    imu_transform->header.stamp = msg->end_timestamp;
    imu_transform->header.frame_id = "body_i-1";
    imu_transform->child_frame_id = "body_i";
    eigen_ros::to_ros(imu_transform->transform, s2s_pose);
    imu_transform_publisher.publish(imu_transform);

    // Add preintegration IMU factor and bias between factor to graph
    timestamps.emplace_back(msg->end_timestamp);
    new_factors.emplace_shared<gtsam::CombinedImuFactor>(X(keys.imu - 1), V(keys.imu - 1), X(keys.imu), V(keys.imu),
            B(keys.imu - 1), B(keys.imu), preintegrated_imu);
    ROS_INFO_STREAM("Added factor: X(" << keys.imu - 1 << "), V(" << keys.imu - 1  << "), B(" << keys.imu - 1 <<
            ") => X(" << keys.imu << "), V(" << keys.imu << "), B(" << keys.imu << ")");
    new_values.insert(X(keys.imu), state.pose());
    ROS_INFO_STREAM("Added value: X(" << keys.imu << ")");
    new_values.insert(V(keys.imu), state.v());
    ROS_INFO_STREAM("Added value: V(" << keys.imu << ")");
    new_values.insert(B(keys.imu), imu_bias);
    ROS_INFO_STREAM("Added value: B(" << keys.imu << ")");
    ++keys.imu;
}

void Optimisation::initial_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard(graph_mutex);

    // Error Checking
    if (keys != Keys()) {
        throw std::runtime_error("Initialisation callback triggered after initialisation. Something went wrong!");
    }

    // Set initial pose prior
    eigen_ros::Odometry odometry = eigen_ros::from_ros<eigen_ros::Odometry>(*msg);
    gtsam::Pose3 prior_pose = gtsam::Pose3(gtsam::Rot3(odometry.pose.orientation), odometry.pose.position);
    auto prior_pose_noise = gtsam::noiseModel::Gaussian::Covariance(reorder_pose_covariance(odometry.pose.covariance));
    ROS_INFO_STREAM("Prior pose sigmas: " << to_flat_string(prior_pose_noise->sigmas()));
    if (prior_pose_noise->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Prior pose noise sigmas contained a zero.");
    }
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), prior_pose, prior_pose_noise);
    ROS_INFO_STREAM("Added prior factor: X(" << 0 << ")");
    new_values.insert(X(0), prior_pose);
    ROS_INFO_STREAM("Added value: X(" << 0 << ")");

    // Set initial velocity prior
    gtsam::Vector3 prior_vel = odometry.twist.linear;
    auto prior_vel_noise = gtsam::noiseModel::Gaussian::Covariance(odometry.twist.covariance.block<3, 3>(0, 0));
    ROS_INFO_STREAM("Prior vel sigmas: " << to_flat_string(prior_vel_noise->sigmas()));
    if (prior_vel_noise->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Prior vel noise sigmas contained a zero.");
    }
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prior_vel, prior_vel_noise);
    ROS_INFO_STREAM("Added prior factor: V(" << 0 << ")");
    new_values.insert(V(0), prior_vel);
    ROS_INFO_STREAM("Added value: V(" << 0 << ")");

    // Set initial bias prior
    ROS_WARN_ONCE("DESIGN DECISION: can bias be estimated from initialisation procedure?");
    const double prior_accel_bias_noise = nh.param<double>("prior_noise/accelerometer_bias", 1.0e-3);
    const double prior_gyro_bias_noise = nh.param<double>("prior_noise/gyroscope_bias", 1.0e-3);
    if (prior_accel_bias_noise == 0.0) {
        throw std::runtime_error("Prior accelerometer bias noise should not be zero.");
    }
    if (prior_gyro_bias_noise == 0.0) {
        throw std::runtime_error("Prior gyroscope bias noise should not be zero.");
    }
    const auto prior_imu_bias_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << prior_accel_bias_noise,
            prior_accel_bias_noise, prior_accel_bias_noise,  prior_gyro_bias_noise, prior_gyro_bias_noise,
            prior_gyro_bias_noise).finished());
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), imu_bias, prior_imu_bias_noise);
    ROS_INFO_STREAM("Added prior factor: B(" << 0 << ")");
    new_values.insert(B(0), imu_bias);
    ROS_INFO_STREAM("Added value: B(" << 0 << ")");
    ROS_WARN_ONCE("DESIGN DECISION: can bias noise be estimated from initialisation procedure?");

    // Save timestamp
    timestamps.emplace_back(msg->header.stamp);

    // Publish empty imu transform (no translation, identity rotation)
    auto imu_transform = boost::make_shared<geometry_msgs::TransformStamped>();
    imu_transform->header = msg->header;
    imu_transform->child_frame_id = msg->child_frame_id;
    eigen_ros::to_ros(imu_transform->transform, eigen_ros::Pose());
    imu_transform_publisher.publish(imu_transform);

    // Publish initial pose as optimised odometry
    optimised_state = gtsam::NavState(gtsam::Rot3(odometry.pose.orientation), odometry.pose.position,
            odometry.twist.linear);
    optimised_odometry_publisher.publish(msg);

    // Publish IMU biases
    auto imu_bias_msg = boost::make_shared<serpent::ImuBiases>();
    to_ros(*imu_bias_msg, imu_bias, msg->header.stamp);
    imu_biases_publisher.publish(imu_bias_msg);

    // Compute and publish optimised path and changed path
    auto global_path = boost::make_shared<nav_msgs::Path>(convert_to_path(new_values, timestamps, 0));
    path_publisher.publish(global_path);
    path_changes_publisher.publish(global_path);
}

void Optimisation::registration_transform_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Add registration transform to graph, update value guess
    const eigen_ros::PoseStamped registration_pose = eigen_ros::from_ros<eigen_ros::PoseStamped>(*msg);
    gtsam::Pose3 registration_pose_gtsam = gtsam::Pose3(gtsam::Rot3(registration_pose.data.orientation),
            registration_pose.data.position);
    graph_mutex.lock();
    if (timestamps.at(keys.pointcloud) != msg->header.stamp) {
        throw std::runtime_error("Timestamps did not match [Graph: " +
                std::to_string(timestamps.at(keys.pointcloud).toSec()) + ", Transform:" +
                std::to_string(msg->header.stamp.toSec()) + "]. Something went wrong.");
    }
    auto registration_covariance = gtsam::noiseModel::Gaussian::Covariance(reorder_pose_covariance(
        registration_pose.data.covariance));
    ROS_INFO_STREAM("Registration sigmas: " << to_flat_string(registration_covariance->sigmas()));
    if (registration_covariance->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Registration noise sigmas contained a zero.");
    }
    new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(keys.pointcloud - 1), X(keys.pointcloud),
            registration_pose_gtsam, registration_covariance);
    ROS_INFO_STREAM("Added factor: X(" << keys.pointcloud - 1 << ") => X(" << keys.pointcloud << ")");
    if (new_values.exists(X(keys.pointcloud))) {
        new_values.update(X(keys.pointcloud), optimised_state.pose() * registration_pose_gtsam);
        ROS_INFO_STREAM("Updated value X(" << keys.pointcloud << ")");
    } else {
        ROS_WARN_STREAM_ONCE("Update of value X(" << keys.pointcloud << ") skipped because value was already added to "
                "iSAM2. Is this behaviour ok?");
    }
    ++keys.pointcloud;

    // Optimise graph with iSAM2
    const gtsam::ISAM2Result optimisation_result = optimiser->update(new_factors, new_values);
    ROS_INFO_STREAM("Optimisation Result: #Reeliminated = " << optimisation_result.getVariablesReeliminated() <<
            ", #Relinearized = " << optimisation_result.getVariablesRelinearized() << ", #Cliques = " <<
            optimisation_result.getCliques());
    const gtsam::Values optimised_values = optimiser->calculateEstimate();
    ROS_INFO_STREAM("Calculated optimiser estimate up for t = " << registration_pose.timestamp);
    const gtsam::Pose3 optimised_pose = optimised_values.at<gtsam::Pose3>(X(keys.pointcloud - 1));
    const Eigen::Vector3d optimised_vel = optimised_values.at<gtsam::Vector3>(V(keys.pointcloud - 1));
    auto optimised_bias = optimised_values.at<gtsam::imuBias::ConstantBias>(B(keys.pointcloud - 1));
    imu_bias = optimised_bias;
    ROS_INFO_STREAM("Optimised Bias:\n" << imu_bias);
    const Eigen::Matrix<double, 6, 6> optimised_pose_covariance = reorder_pose_covariance(
            optimiser->marginalCovariance(X(keys.pointcloud - 1)));
    const Eigen::Matrix3d optimised_vel_covariance = optimiser->marginalCovariance(V(keys.pointcloud - 1));
    auto global_path = boost::make_shared<nav_msgs::Path>(convert_to_path(optimised_values, timestamps,
            keys.pointcloud - 1));
    // Resetting factors and values for next update
    new_factors = gtsam::NonlinearFactorGraph();
    new_values = gtsam::Values();
    ROS_INFO_STREAM("Reset factors and values");
    graph_mutex.unlock();

    // Publish optimised pose
    auto optimised_odometry_msg = boost::make_shared<nav_msgs::Odometry>();
    optimised_odometry_msg->header.stamp = msg->header.stamp;
    optimised_odometry_msg->header.frame_id = "map";
    optimised_odometry_msg->child_frame_id = "body_i-1";
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.position, optimised_pose.translation());
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.orientation, optimised_pose.rotation().toQuaternion());
    eigen_ros::to_ros(optimised_odometry_msg->pose.covariance, optimised_pose_covariance);
    eigen_ros::to_ros(optimised_odometry_msg->twist.twist.linear, optimised_vel);
    ROS_WARN_ONCE("TODO: add angular velocity from IMU odometry to optimised odometry");
    ROS_WARN_ONCE("TODO: add twist covariance from IMU odometry to optimised odometry");
    optimised_odometry_publisher.publish(optimised_odometry_msg);

    // Publish optimised biases
    auto imu_bias_msg = boost::make_shared<serpent::ImuBiases>();
    to_ros(*imu_bias_msg, optimised_bias, msg->header.stamp);
    imu_biases_publisher.publish(imu_bias_msg);

    // Publish optimised path and changed path
    auto global_path_changes = boost::make_shared<nav_msgs::Path>(extract_changed_poses(*global_path,
            optimisation_result));
    path_publisher.publish(global_path);
    path_changes_publisher.publish(global_path_changes);
}

nav_msgs::Path convert_to_path(const gtsam::Values& values, const std::vector<ros::Time> timestamps,
        const gtsam::Key max_key) {
    if (max_key >= timestamps.size()) {
        throw std::runtime_error("Max key greater than timestamps size. Something went wrong.");
    }
    nav_msgs::Path path;
    path.header.stamp = timestamps[max_key];
    path.header.frame_id = "map";
    for (gtsam::Key key = 0; key <= max_key; ++key) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = timestamps[key];
        pose_msg.header.frame_id = "body_" + std::to_string(key);
        to_ros(pose_msg.pose, values.at<gtsam::Pose3>(X(key)));
        path.poses.push_back(pose_msg);
    }
    return path;
}

nav_msgs::Path extract_changed_poses(const nav_msgs::Path& full_path, const gtsam::ISAM2Result& optimisation_result) {
    nav_msgs::Path path_changes;
    path_changes.header = full_path.header;
    for (const auto& pose : full_path.poses) {
        ROS_WARN_ONCE("TODO FIX: use gtsam::ISAM2Result to check if pose has changed");
        path_changes.poses.push_back(pose);
    }
    return path_changes;
}

}
