#include "serpent/optimisation.hpp"
#include "serpent/utilities.hpp"
#include "serpent/ImuBiases.h"
#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>
#include <eigen_ros/eigen_ros.hpp>

namespace serpent {

void from_ros(const std::vector<serpent::StereoLandmark>& msgs, std::map<int, gtsam::StereoPoint2>& landmarks) {
    for (const auto& msg : msgs) {
        const double mean_y = (msg.left_y + msg.right_y) / 2.0;
        if (!landmarks.emplace(msg.id, gtsam::StereoPoint2{msg.left_x, msg.right_x, mean_y}).second) {
            throw std::runtime_error("Failed to convert landmark. This could indicate a duplicate id.");
        }
    }
}

Optimisation::Optimisation():
    nh("~"), opt_sync(10)
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

    // Factor Configuration
    nh.param<bool>("optimisation/factors/imu", imu_factors_enabled, true);
    nh.param<bool>("optimisation/factors/registration", registration_factors_enabled, true);
    nh.param<bool>("optimisation/factors/stereo", stereo_factors_enabled, true);
    ROS_INFO_STREAM("IMU Factors " << (imu_factors_enabled ? "ENABLED" : "DISABLED"));
    ROS_INFO_STREAM("Registration Factors " << (registration_factors_enabled ? "ENABLED" : "DISABLED"));
    if (!imu_factors_enabled && !registration_factors_enabled) {
        throw std::runtime_error("Must have at least one of imu and registration factors enabled");
    }

    // Optimisation subscribers
    if (registration_factors_enabled && stereo_factors_enabled) {
        registration_filter_subscriber.subscribe(nh, "registration/transform", 10);
        stereo_landmarks_filter_subscriber.subscribe(nh, "stereo/landmarks", 10);
        opt_sync.connectInput(registration_filter_subscriber, stereo_landmarks_filter_subscriber);
        opt_sync.registerCallback(boost::bind(&Optimisation::registration_stereo_landmarks_callback, this, _1, _2));
    } else if (registration_factors_enabled) {
        registration_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("registration/transform", 10,
                &Optimisation::registration_callback, this);
    } else if (stereo_factors_enabled) {
        stereo_landmarks_subscriber = nh.subscribe<serpent::StereoLandmarks>("stereo/landmarks", 10,
                &Optimisation::stereo_landmarks_callback, this);
    }
    
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
    // pose of the sensor in the body frame
    const gtsam::Pose3 body_to_imu = eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame("imu"));
    preintegration_params->setBodyPSensor(body_to_imu);
    
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
    if (isam2_params.isEvaluateNonlinearError()) {
        ROS_WARN("Evaluation of Nonlinear Error in iSAM2 optimisation is ENABLED (isam2/evaluate_nonlinear_error ="
                " true). This incurs a computational cost, so should only be used while debugging.");
    }
    isam2_params.print();

    // Set up GraphManager
    gm = std::make_unique<ISAM2GraphManager>(isam2_params);
    gm->set_named_key("imu");
    gm->set_named_key("reg");
    if (stereo_factors_enabled) {
        gm->set_named_key("stereo", -1);
        const std::string stereo_left_cam_frame = nh.param<std::string>("stereo_factors/left_cam_frame_id", "stereo");
        gm->set_body_to_stereo_left_cam_pose(eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame(
                stereo_left_cam_frame)));
        auto stereo_measurement_covariance = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
                nh.param<double>("stereo_factors/noise/left_x", 5.0),
                nh.param<double>("stereo_factors/noise/right_x", 5.0),
                nh.param<double>("stereo_factors/noise/y", 10.0)).finished());
        ROS_INFO_STREAM("Set stereo measurement covariance:\n" << stereo_measurement_covariance->covariance());
        gm->set_stereo_measurement_covariance(stereo_measurement_covariance);
    }
}

void Optimisation::add_registration_factor(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    gm->increment("reg");
    // Extract registration information
    const eigen_ros::PoseStamped registration_pose = eigen_ros::from_ros<eigen_ros::PoseStamped>(*msg);
    const gtsam::Pose3 registration_pose_gtsam = gtsam::Pose3(gtsam::Rot3(registration_pose.data.orientation),
            registration_pose.data.position);
    auto registration_covariance = gtsam::noiseModel::Gaussian::Covariance(registration_pose.data.covariance);
    ROS_INFO_STREAM("Registration covariance sigmas: " << to_flat_string(registration_covariance->sigmas()));
    if (registration_covariance->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Registration noise sigmas contained a zero.");
    }

    // Update state
    gm->set_pose("reg", gm->pose("reg", -1) * registration_pose_gtsam);
    if (gm->timestamp("reg") != msg->header.stamp) {
        throw std::runtime_error("Timestamps did not match [Graph: " + std::to_string(gm->timestamp("reg").toSec())
                + ", Transform:" + std::to_string(msg->header.stamp.toSec()) + "]. Something went wrong.");
    }
    
    // Add the registration factor
    const int reg_key = gm->key("reg");
    gm->create_between_pose_factor(reg_key, registration_pose_gtsam, registration_covariance);
    ROS_INFO_STREAM("Created factor: X(" << reg_key - 1 << ") => X(" << reg_key << ")");
}

void Optimisation::add_stereo_factors(const serpent::StereoLandmarks::ConstPtr& landmarks) {
    gm->increment("stereo");
    if (!gm->stereo_calibration()) {
        const auto& left_info = landmarks->left_info;
        const double fx = left_info.K[0];
        const double skew = left_info.K[1];
        const double cx = left_info.K[2];
        const double fy = left_info.K[4];
        const double cy = left_info.K[5];
        const double baseline = nh.param<double>("stereo/baseline", 0.1);
        auto K = boost::make_shared<gtsam::Cal3_S2Stereo>(fx, fy, skew, cx, cy, baseline);
        gm->set_stereo_calibration(K);
        ROS_INFO_STREAM("Set stereo calibration matrix:\n" << K->matrix());
    }
    std::map<int, gtsam::StereoPoint2> stereo_landmarks;
    from_ros(landmarks->landmarks, stereo_landmarks);
    gm->add_stereo_landmark_measurements(gm->key("stereo"), stereo_landmarks);
    ROS_INFO_STREAM("Added " << stereo_landmarks.size() << " stereo landmarks measurements to graph manager");
}

void Optimisation::imu_s2s_callback(const serpent::ImuArray::ConstPtr& msg) {
    // Deserialise imu array
    std::deque<eigen_ros::Imu> imu_s2s;
    from_ros(msg->measurements, imu_s2s);
    if (imu_s2s.empty()) {
        throw std::runtime_error("S2S imu array was empty");
    }

    // Wait for imu bias to be optimised (when optimisation has run)
    if (!protected_sleep(graph_mutex, 0.01, false, true, [this]()
            { return gm->opt_key() != gm->key("imu"); })) {
        return;
    };

    // Create IMU preintegration
    update_preintegration_params(*preintegration_params, imu_s2s.front().linear_acceleration_covariance,
            imu_s2s.front().angular_velocity_covariance);
    gtsam::PreintegratedCombinedMeasurements preintegrated_imu{preintegration_params, gm->imu_bias("imu")};

    // Preintegrate IMU measurements over sweep
    ros::Time last_preint_imu_timestamp = msg->start_timestamp;
    for (std::size_t i = 0; i < imu_s2s.size(); ++i) {
        const auto& imu = imu_s2s[i];
        if (imu.timestamp < last_preint_imu_timestamp || imu.timestamp > msg->end_timestamp) {
            throw std::runtime_error("IMU timestamp " + std::to_string(imu.timestamp.toSec()) + " was outside of "
                    "expected range [" + std::to_string(last_preint_imu_timestamp.toSec()) + " - "
                    + std::to_string(msg->end_timestamp.toSec()) + "]");
        }
        const double dt = ((i == imu_s2s.size() - 1 ? msg->end_timestamp : imu.timestamp)
                - last_preint_imu_timestamp).toSec();
        if (dt > 0.0) {
            preintegrated_imu.integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
        }
        last_preint_imu_timestamp = imu.timestamp;
    }
    ROS_INFO_STREAM("Preintegrated " << imu_s2s.size() << " IMU messages between t = " << msg->start_timestamp
            << " and t = " << msg->end_timestamp);
    if (preintegrated_imu.preintMeasCov().hasNaN()) {
        ROS_WARN_STREAM("Preintegrated Measurement Covariance:\n" << preintegrated_imu.preintMeasCov());
        throw std::runtime_error("NaN found in preintegrated measurement covariance. Something went wrong.");
    } else if (preintegrated_imu.preintMeasCov().diagonal().minCoeff() <= 0.0) {
        ROS_WARN_STREAM("Preintegrated Measurement Covariance:\n" << preintegrated_imu.preintMeasCov());
        throw std::runtime_error("Minimum coefficient in preintegrated measurement covariance was <= 0.0");
    }
    
    // Predict new pose from preintegration
    gm->increment("imu");
    gm->set_imu_bias("imu", gm->imu_bias("imu", -1));
    gm->set_navstate("imu", preintegrated_imu.predict(gm->navstate("imu", -1), gm->imu_bias("imu", -1)));
    gm->set_timestamp("imu", msg->end_timestamp);
    if (gm->pose("imu").matrix().hasNaN()) {
        ROS_WARN_STREAM("Preintegrated pose estimate:\n" << gm->pose("imu").matrix());
        throw std::runtime_error("NaN found in preintegrated pose estimate.");
    }
    if (gm->velocity("imu").hasNaN()) {
        ROS_WARN_STREAM("Preintegrated velocity estimate:" << to_flat_string(gm->velocity("imu")));
        throw std::runtime_error("NaN found in preintegrated velocity estimate.");
    }

    // Calculate change in pose: T_{B_i-1}^{B_i} = T_{B_i-1}^W * T_W^{B_i} = (T_W^{B_i-1})^-1 * T_W^{B_i}
    // Note that result is in body frame because gm stores poses in body frame
    const Eigen::Isometry3d s2s_pose_body = Eigen::Isometry3d(gm->pose("imu", -1).matrix()).inverse()
            * Eigen::Isometry3d(gm->pose("imu").matrix());
    // Convert to the lidar frame
    const eigen_ros::Pose s2s_pose_lidar{eigen_ext::change_relative_transform_frame(s2s_pose_body,
            body_frames.frame_to_body("lidar"))};
    ROS_WARN_ONCE("DESIGN DECISION: change output to odometry to pass velocity & covs to registration module?");

    // Publish imu estimated transform
    auto imu_transform = boost::make_shared<geometry_msgs::TransformStamped>();
    imu_transform->header.stamp = gm->timestamp("imu"); // Timestamp must be that of the child frame for synchronisation
    imu_transform->header.frame_id = "lidar_i-1"; // i-1
    imu_transform->child_frame_id = "lidar"; // i
    eigen_ros::to_ros(imu_transform->transform, s2s_pose_lidar);
    imu_transform_publisher.publish(imu_transform);

    // Add preintegration combined IMU factor (includes bias between factor) to graph
    if (imu_factors_enabled) {
        const int imu_key = gm->key("imu");
        gm->create_combined_imu_factor(imu_key, preintegrated_imu);
        ROS_INFO_STREAM("Created factor: X(" << imu_key - 1 << "), V(" << imu_key - 1  << "), B(" << imu_key - 1 <<
                ") => X(" << imu_key << "), V(" << imu_key << "), B(" << imu_key << ")");

        // If no other factors, optimise
        if (!registration_factors_enabled && !stereo_factors_enabled) {
            optimise_and_publish(imu_key);
        }
    }

    graph_mutex.unlock();
}

void Optimisation::initial_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Initial odometry state
    const eigen_ros::Odometry odometry = eigen_ros::from_ros<eigen_ros::Odometry>(*msg);

    // State manager
    std::lock_guard<std::mutex> guard(graph_mutex);
    gm->set_imu_bias(0, gtsam::imuBias::ConstantBias());
    ROS_WARN_ONCE("DESIGN DECISION: can bias be estimated from initialisation procedure?");
    gm->set_pose(0, gtsam::Pose3(gtsam::Rot3(odometry.pose.orientation), odometry.pose.position));
    gm->set_timestamp(0, msg->header.stamp);
    gm->set_velocity(0, odometry.twist.linear);

    // Prior noises
    const auto prior_pose_noise = gtsam::noiseModel::Gaussian::Covariance(odometry.pose.covariance);
    ROS_INFO_STREAM("Prior pose sigmas: " << to_flat_string(prior_pose_noise->sigmas()));
    if (prior_pose_noise->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Prior pose noise sigmas contained a zero.");
    }
    gm->create_prior_pose_factor(0, gm->pose(0), prior_pose_noise);
    ROS_INFO_STREAM("Created prior factor: X(" << 0 << ")");

    if (imu_factors_enabled) {
        // Prior velocity noise
        auto prior_velocity_noise = gtsam::noiseModel::Gaussian::Covariance(
                odometry.twist.linear_velocity_covariance());
        ROS_INFO_STREAM("Prior vel sigmas: " << to_flat_string(prior_velocity_noise->sigmas()));
        if (prior_velocity_noise->sigmas().minCoeff() == 0.0) {
            throw std::runtime_error("Prior vel noise sigmas contained a zero.");
        }
        gm->create_prior_velocity_factor(0, gm->velocity(0), prior_velocity_noise);
        ROS_INFO_STREAM("Created prior factor: V(" << 0 << ")");

        // Prior imu noise
        const double prior_accel_bias_noise = nh.param<double>("prior_noise/accelerometer_bias", 1.0e-3);
        const double prior_gyro_bias_noise = nh.param<double>("prior_noise/gyroscope_bias", 1.0e-3);
        if (prior_accel_bias_noise == 0.0) {
            throw std::runtime_error("Prior accelerometer bias noise should not be zero.");
        }
        if (prior_gyro_bias_noise == 0.0) {
            throw std::runtime_error("Prior gyroscope bias noise should not be zero.");
        }
        auto prior_imu_bias_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << prior_accel_bias_noise,
                prior_accel_bias_noise, prior_accel_bias_noise, prior_gyro_bias_noise, prior_gyro_bias_noise,
                prior_gyro_bias_noise).finished());
        ROS_INFO_STREAM("Prior bias sigmas: " << to_flat_string(prior_imu_bias_noise->sigmas()));
        gm->create_prior_imu_bias_factor(0, gm->imu_bias(0), prior_imu_bias_noise);
        ROS_INFO_STREAM("Created prior factor: B(" << 0 << ")");
    }

    // Publish empty imu transform (no translation, identity rotation)
    auto imu_transform = boost::make_shared<geometry_msgs::TransformStamped>();
    imu_transform->header = msg->header;
    imu_transform->child_frame_id = msg->child_frame_id;
    eigen_ros::to_ros(imu_transform->transform, eigen_ros::Pose());
    imu_transform_publisher.publish(imu_transform);

    // Optimise and publish
    optimise_and_publish(0);
}

void Optimisation::optimise_and_publish(const int key) {
    // Optimise graph with iSAM2
    const ros::WallTime tic = ros::WallTime::now();
    const gtsam::ISAM2Result isam2_result = gm->optimise(key);
    ROS_INFO_STREAM("Optimised and calculated estimate for key = " << key << ", t = " << gm->timestamp(key) << " in "
            << (ros::WallTime::now() - tic).toSec() << " seconds.\n#Reeliminated = "
            << isam2_result.getVariablesReeliminated() << ", #Relinearized = "
            << isam2_result.getVariablesRelinearized() << ", #Cliques = " << isam2_result.getCliques()
            << ", Factors Recalculated = " << isam2_result.factorsRecalculated
            << (isam2_result.errorBefore && isam2_result.errorAfter ? "\nError Before = "
            + std::to_string(*isam2_result.errorBefore) + ", Error After = "
            + std::to_string(*isam2_result.errorAfter) : ""));

    // Publish optimised pose
    auto optimised_odometry_msg = boost::make_shared<nav_msgs::Odometry>();
    optimised_odometry_msg->header.stamp = gm->timestamp(key);
    optimised_odometry_msg->header.frame_id = "map";
    optimised_odometry_msg->child_frame_id = body_frames.body_frame();
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.position, gm->pose(key).translation());
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.orientation, gm->pose(key).rotation().toQuaternion());
    eigen_ros::to_ros(optimised_odometry_msg->pose.covariance, eigen_ext::reorder_covariance(
            gm->pose_covariance(key), 3));
    eigen_ros::to_ros(optimised_odometry_msg->twist.twist.linear, gm->velocity(key));
    ROS_WARN_ONCE("TODO: add angular velocity from IMU odometry to optimised odometry");
    const Eigen::Matrix3d optimised_vel_covariance = imu_factors_enabled ? gm->velocity_covariance(key)
            : Eigen::Matrix3d::Zero();
    ROS_WARN_ONCE("TODO: add linear velocity covariance (optimised_vel_covariance) to optimised odometry");
    ROS_WARN_ONCE("TODO: add angular velocity covariance from IMU odometry to optimised odometry");
    optimised_odometry_publisher.publish(optimised_odometry_msg);
    ROS_INFO_STREAM("Pose:\n" << gm->pose(key).matrix());
    ROS_INFO_STREAM("Pose Covariance (r, p, y, x, y, z):\n" << gm->pose_covariance(key));
    if (imu_factors_enabled) {
        ROS_INFO_STREAM("Velocity: " << to_flat_string(gm->velocity(key)));
        ROS_INFO_STREAM("Velocity Covariance:\n" << optimised_vel_covariance);
        ROS_INFO_STREAM("IMU Bias:\n" << gm->imu_bias(key));
        ROS_INFO_STREAM("IMU Bias Covariance:\n" << gm->imu_bias_covariance(key));
    }
    
    // Publish optimised biases
    auto imu_bias_msg = boost::make_shared<serpent::ImuBiases>();
    to_ros(*imu_bias_msg, gm->imu_bias(key), gm->timestamp(key));
    imu_biases_publisher.publish(imu_bias_msg);

    // Publish optimised path and changed path
    auto global_path = boost::make_shared<nav_msgs::Path>(convert_to_path(*gm, key, body_frames.body_frame()));
    auto global_path_changes = boost::make_shared<nav_msgs::Path>(extract_changed_poses(*global_path, isam2_result));
    path_publisher.publish(global_path);
    path_changes_publisher.publish(global_path_changes);
}

void Optimisation::registration_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Add registration transform to graph, update value guess
    std::lock_guard<std::mutex> guard{graph_mutex};

    // Integrate registration data
    add_registration_factor(msg);

    // Optimise and publish
    optimise_and_publish(gm->key("reg"));

    // Update values that weren't optimised
    if (!imu_factors_enabled) {
        update_velocity_from_transforms("reg");
    }
}

void Optimisation::registration_stereo_landmarks_callback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& registration,
        const serpent::StereoLandmarks::ConstPtr& landmarks) {
    std::lock_guard<std::mutex> guard{graph_mutex};
    
    // Integrate registration data
    add_registration_factor(registration);

    // Integrate stereo data
    add_stereo_factors(landmarks);

    // Optimise and publish
    optimise_and_publish(gm->key("reg"));

    // Update velocity if no IMU factors
    if (!imu_factors_enabled) {
        update_velocity_from_transforms("reg");
    }
}

void Optimisation::stereo_landmarks_callback(const serpent::StereoLandmarks::ConstPtr& landmarks) {
    // Wait until IMU data has been processed
    ROS_INFO_STREAM("Waiting for IMU data to be processed by graph manager");
    if (!protected_sleep(graph_mutex, 0.01, false, true, [this](){ return gm->key("imu") <= gm->key("stereo"); })) {
        return;
    }
    ROS_INFO_STREAM("Finished waiting for IMU data to be processed by graph manager");

    // Integrate stereo data
    add_stereo_factors(landmarks);

    // Optimise and publish
    if (gm->key("stereo") > 0) {
        optimise_and_publish(gm->key("stereo"));

        // Update velocity if no IMU factors
        if (!imu_factors_enabled) {
            update_velocity_from_transforms("stereo");
        }
    }
    
    graph_mutex.unlock();
}

void Optimisation::update_velocity_from_transforms(const std::string& named_key) {
    // Set velocity based on transforms
    const double dt = gm->time_between(named_key, named_key, -1).toSec();
    Eigen::Matrix<double, 6, 1> rates = eigen_ext::linear_rates(
            eigen_gtsam::to_eigen<Eigen::Isometry3d>(gm->pose(named_key, -1)),
            eigen_gtsam::to_eigen<Eigen::Isometry3d>(gm->pose(named_key)), dt);
    ROS_INFO_STREAM("Estimated twist [ang, lin] from transforms as:\n" << to_flat_string(rates));
    const Eigen::Vector3d linear_velocity = rates.block<3, 1>(3, 0);
    gm->set_velocity(named_key, linear_velocity);
}

nav_msgs::Path convert_to_path(const GraphManager& gm, const int max_key, const std::string& frame_id_prefix) {
    nav_msgs::Path path;
    path.header.stamp = gm.timestamp(max_key);
    path.header.frame_id = "map";
    for (int key = 0; key <= max_key; ++key) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = gm.timestamp(key);
        pose_msg.header.frame_id = frame_id_prefix + "_" + std::to_string(key);
        to_ros(pose_msg.pose, gm.pose(key));
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
