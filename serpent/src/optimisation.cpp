#include "serpent/optimisation.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <pcl_ros/point_cloud.h>

#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>
#include <eigen_ros/eigen_ros.hpp>

#include "serpent/ImuBiases.h"
#include "serpent/utilities.hpp"

namespace serpent {

void from_ros(const std::vector<serpent::StereoFeature>& msgs, std::map<int, gtsam::StereoPoint2>& features) {
    for (const auto& msg : msgs) {
        const double mean_y = (msg.left_y + msg.right_y) / 2.0;
        if (!features.emplace(msg.id, gtsam::StereoPoint2{msg.left_x, msg.right_x, mean_y}).second) {
            throw std::runtime_error("Failed to convert stereo feature. This could indicate a duplicate id.");
        }
    }
}

void to_pcl(const std::map<int, gtsam::Point3>& points, pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
    for (const auto& [id, point] : points) {
        pointcloud.push_back(pcl::PointXYZ{static_cast<float>(point.x()), static_cast<float>(point.y()),
                static_cast<float>(point.z())});
    }
}

void to_pcl(const std::vector<gtsam::Point3>& points, pcl::PointCloud<pcl::PointXYZ>& pointcloud) {
    for (const gtsam::Point3& point : points) {
        pointcloud.push_back(pcl::PointXYZ{static_cast<float>(point.x()), static_cast<float>(point.y()),
                static_cast<float>(point.z())});
    }
}

gtsam::noiseModel::mEstimator::Base::ReweightScheme to_reweight_scheme(const std::string& reweight_scheme) {
    if (reweight_scheme == "BLOCK") {
        return gtsam::noiseModel::mEstimator::Base::ReweightScheme::Block;
    } else if (reweight_scheme == "SCALAR") {
        return gtsam::noiseModel::mEstimator::Base::ReweightScheme::Scalar;
    }
    throw std::runtime_error("ReweightScheme \'" + reweight_scheme + "\' not recognised.");
}

Optimisation::Optimisation()
    : nh("~") {
    // Publishers
    imu_biases_publisher = nh.advertise<serpent::ImuBiases>("optimisation/imu_biases", 1);
    imu_transform_publisher = nh.advertise<geometry_msgs::TransformStamped>("optimisation/imu_transform", 1);
    optimised_odometry_publisher = nh.advertise<nav_msgs::Odometry>("optimisation/odometry", 1);
    path_publisher = nh.advertise<nav_msgs::Path>("output/path", 1);
    path_changes_publisher = nh.advertise<nav_msgs::Path>("optimisation/path_changes", 1);

    // Subscribers
    imu_s2s_subscriber = nh.subscribe<serpent::ImuArray>("frontend/imu_s2s", 10, &Optimisation::imu_s2s_callback, this);
    initial_odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("frontend/initial_odometry", 1,
            &Optimisation::initial_odometry_callback, this);

    // Factor Configuration
    nh.param<bool>("optimisation/factors/imu", imu_factors_enabled, true);
    nh.param<bool>("optimisation/factors/registration", registration_factors_enabled, true);
    nh.param<bool>("optimisation/factors/stereo", stereo_factors_enabled, true);
    ROS_INFO_STREAM("Factors:\nIMU            " << (imu_factors_enabled ? "ENABLED" : "DISABLED") << "\nRegistration   "
                                                << (registration_factors_enabled ? "ENABLED" : "DISABLED")
                                                << "\nStereo         "
                                                << (stereo_factors_enabled ? "ENABLED" : "DISABLED"));

    // Optimisation subscribers
    if (registration_factors_enabled) {
        registration_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("registration/transform", 10,
                &Optimisation::registration_callback, this);
    } else if (stereo_factors_enabled) {
        stereo_features_subscriber = nh.subscribe<serpent::StereoFeatures>("stereo/features", 10,
                &Optimisation::stereo_features_callback, this);
    }

    // Debugging
    nh.param<bool>("debug/optimisation/publish_stereo_points", publish_stereo_points, false);

    // Debugging publishers
    if (publish_stereo_points) {
        stereo_points_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("debug/stereo_points", 1);
    }

    // Preintegration parameters
    preintegration_params = gtsam::PreintegrationCombinedParams::MakeSharedU(nh.param<double>("gravity", 9.81));
    ROS_WARN_ONCE("DESIGN DECISION: gravity from initialisation procedure?");
    preintegration_params->setIntegrationCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu_noise/integration", 1.0e-3), 2.0));
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
                                              std::pow(nh.param<double>("imu_noise/integration_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasAccCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu_noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu_noise/gyroscope_bias", 1.0e-3), 2.0));
    // pose of the sensor in the body frame
    const gtsam::Pose3 body_to_imu = eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame("imu"));
    preintegration_params->setBodyPSensor(body_to_imu);

    // Optimiser
    gtsam::ISAM2Params isam2_params;
    const std::string optimization = nh.param<std::string>("isam2/optimization", "GaussNewton");
    if (optimization == "GaussNewton") {
        gtsam::ISAM2GaussNewtonParams optimization_params = gtsam::ISAM2GaussNewtonParams();
        optimization_params.setWildfireThreshold(
                nh.param<double>("isam2/gauss_newton_params/wildfire_threshold", 1.0e-3));
        isam2_params.setOptimizationParams(optimization_params);
    } else if (optimization == "Dogleg") {
        gtsam::ISAM2DoglegParams optimization_params = gtsam::ISAM2DoglegParams();
        optimization_params.setInitialDelta(nh.param<double>("isam2/dogleg_params/initial_delta", 1.0));
        optimization_params.setWildfireThreshold(nh.param<double>("isam2/dogleg_params/wildfire_threshold", 1.0e-5));
        optimization_params.setAdaptationMode(
                nh.param<std::string>("isam2/dogleg_params/trust_region_adaptation_mode", "SEARCH_EACH_ITERATION"));
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
        ROS_WARN("Evaluation of Nonlinear Error in iSAM2 optimisation is ENABLED (isam2/evaluate_nonlinear_error = "
                 "true). This incurs a computational cost, so should only be used while debugging.");
    }
    isam2_params.print();

    // Set up GraphManager
    gm = std::make_unique<ISAM2GraphManager>(isam2_params);
    gm->set_named_key("imu");
    if (registration_factors_enabled) {
        gm->set_named_key("reg");
    }
    if (stereo_factors_enabled) {
        gm->set_named_key("stereo", -1);

        // Stereo reference frame
        const std::string stereo_left_cam_frame = nh.param<std::string>("stereo_factors/left_cam_frame", "stereo");
        gm->set_body_to_stereo_left_cam_pose(
                eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame(stereo_left_cam_frame)));

        // Create robust noise model if required
        auto stereo_measurement_covariance = gtsam::noiseModel::Diagonal::Sigmas((
                gtsam::Vector(3) << nh.param<double>("stereo_factors/noise/left_x", 5.0),
                nh.param<double>("stereo_factors/noise/right_x", 5.0), nh.param<double>("stereo_factors/noise/y", 10.0))
                                                                                         .finished());
        ROS_INFO_STREAM("Set stereo measurement covariance:\n" << stereo_measurement_covariance->covariance());
        gtsam::SharedNoiseModel stereo_noise_model = stereo_measurement_covariance;
        if (nh.param<bool>("stereo_factors/robust/enabled", true)) {
            gtsam::noiseModel::mEstimator::Base::shared_ptr robust_noise_model;
            const std::string robust_noise_type = nh.param<std::string>("stereo_factors/robust/type", "huber");
            if (robust_noise_type == "cauchy") {
                const double k = nh.param<double>("stereo_factors/robust/cauchy/k", 0.1);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/cauchy/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::Cauchy::Create(k, reweight_scheme);
            } else if (robust_noise_type == "dcs") {
                const double c = nh.param<double>("stereo_factors/robust/dcs/c", 1.0);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme =
                        to_reweight_scheme(nh.param<std::string>("stereo_factors/robust/dcs/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::DCS::Create(c, reweight_scheme);
            } else if (robust_noise_type == "fair") {
                const double c = nh.param<double>("stereo_factors/robust/fair/c", 1.3998);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/fair/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::Fair::Create(c, reweight_scheme);
            } else if (robust_noise_type == "geman_mcclure") {
                const double c = nh.param<double>("stereo_factors/robust/geman_mcclure/c", 1.0);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/geman_mcclure/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::GemanMcClure::Create(c, reweight_scheme);
            } else if (robust_noise_type == "huber") {
                const double k = nh.param<double>("stereo_factors/robust/huber/k", 1.345);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/huber/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::Huber::Create(k, reweight_scheme);
            } else if (robust_noise_type == "l2_with_dead_zone") {
                const double k = nh.param<double>("stereo_factors/robust/l2_with_dead_zone/k", 1.0);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/l2_with_dead_zone/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(k, reweight_scheme);
            } else if (robust_noise_type == "null") {
                robust_noise_model = gtsam::noiseModel::mEstimator::Null::Create();
            } else if (robust_noise_type == "tukey") {
                const double c = nh.param<double>("stereo_factors/robust/tukey/c", 4.6851);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/tukey/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::Tukey::Create(c, reweight_scheme);
            } else if (robust_noise_type == "welsch") {
                const double c = nh.param<double>("stereo_factors/robust/welsch/c", 2.9846);
                const gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight_scheme = to_reweight_scheme(
                        nh.param<std::string>("stereo_factors/robust/welsch/reweight_scheme", "BLOCK"));
                robust_noise_model = gtsam::noiseModel::mEstimator::Welsch::Create(c, reweight_scheme);
            }
            stereo_noise_model = gtsam::noiseModel::Robust::Create(robust_noise_model, stereo_measurement_covariance);
            ROS_INFO_STREAM("Created robust stereo noise model of type \'" << robust_noise_type << "\'");
        }
        gm->set_stereo_noise_model(stereo_noise_model);
    }
}

void Optimisation::add_registration_factor(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    gm->increment("reg");
    if (gm->timestamp("reg") != msg->header.stamp) {
        throw std::runtime_error("Registrations are out of sync with graph manager [Graph timestamp: " +
                                 std::to_string(gm->timestamp("reg").toSec()) +
                                 ", Transform timestamp:" + std::to_string(msg->header.stamp.toSec()) + "].");
    }

    // Extract registration information (note that eigen_ros reorders covariance from ROS to eigen_ros/GTSAM convention)
    const eigen_ros::PoseStamped registration_pose = eigen_ros::from_ros<eigen_ros::PoseStamped>(*msg);
    const gtsam::Pose3 registration_pose_gtsam =
            gtsam::Pose3(gtsam::Rot3(registration_pose.data.orientation), registration_pose.data.position);

    // Check if valid covariance matrix
    if (eigen_ext::is_valid_covariance(registration_pose.data.covariance)) {
        ROS_ERROR_STREAM("Registration covariance is invalid:\n" << registration_pose.data.covariance);
        throw std::runtime_error("Covariance matrix is not valid.");
    }

    // Convert to GTSAM noise model
    auto registration_covariance = gtsam::noiseModel::Gaussian::Covariance(registration_pose.data.covariance);
    ROS_INFO_STREAM("Registration covariance:\n" << registration_covariance->covariance());
    ROS_INFO_STREAM("Registration covariance sigmas (r, t): " << to_flat_string(registration_covariance->sigmas()));
    if (registration_covariance->sigmas().minCoeff() == 0.0) {
        throw std::runtime_error("Registration noise sigmas contained a zero.");
    }

    // Update state
    gm->set_pose("reg", gm->pose("reg", -1) * registration_pose_gtsam);

    // Add the registration factor
    const int reg_key = gm->key("reg");
    gm->create_between_pose_factor(reg_key, registration_pose_gtsam, registration_covariance);
    ROS_INFO_STREAM("Created factor: X(" << reg_key - 1 << ") => X(" << reg_key << ")");
}

void Optimisation::add_stereo_factors(const serpent::StereoFeatures::ConstPtr& features) {
    gm->increment("stereo");
    if (gm->timestamp("stereo") != features->header.stamp) {
        throw std::runtime_error("Stereo features are out of sync with graph manager [Graph timestamp: " +
                                 std::to_string(gm->timestamp("stereo").toSec()) +
                                 ", Features timestamp:" + std::to_string(features->header.stamp.toSec()) + "].");
    }

    // Initialise stereo calibration
    if (!gm->stereo_calibration()) {
        const auto& left_info = features->left_info;
        const auto& right_info = features->right_info;
        const double fx = left_info.K[0];
        const double skew = left_info.K[1];
        const double cx = left_info.K[2];
        const double fy = left_info.K[4];
        const double cy = left_info.K[5];
        const double baseline = -right_info.P[3] / right_info.P[0];  // Tx = -fx * b => b = -Tx / fx
        if (baseline <= 0.0) {
            throw std::runtime_error("Invalid stereo baseline: " + std::to_string(baseline) +
                                     " (does the right camera info have Tx set correctly in its projection matrix?)");
        }
        auto K = boost::make_shared<gtsam::Cal3_S2Stereo>(fx, fy, skew, cx, cy, baseline);
        gm->set_stereo_calibration(K);
        ROS_INFO_STREAM("Set stereo calibration matrix (baseline: " << K->baseline() << "):\n" << K->matrix());
    }

    // Generate stereo factors and values
    std::map<int, gtsam::StereoPoint2> stereo_features;
    from_ros(features->features, stereo_features);
    gm->create_stereo_factors_and_values(gm->key("stereo"), stereo_features);
    ROS_INFO_STREAM(
            "Added " << stereo_features.size() << " stereo measurements to graph manager at key " << gm->key("stereo"));
}

void Optimisation::imu_s2s_callback(const serpent::ImuArray::ConstPtr& msg) {
    // Deserialise imu array
    std::deque<eigen_ros::Imu> imu_s2s;
    from_ros(msg->measurements, imu_s2s);
    if (imu_s2s.empty()) {
        throw std::runtime_error("S2S imu array was empty");
    }

    // Wait for imu bias to be optimised (when optimisation has run)
    if (!protected_sleep(graph_mutex, 0.01, false, true, [this]() { return gm->opt_key() != gm->key("imu"); })) {
        return;
    };

    // Create IMU preintegration
    update_preintegration_params(*preintegration_params, imu_s2s.front().linear_acceleration_covariance,
            imu_s2s.front().angular_velocity_covariance);
    gtsam::PreintegratedCombinedMeasurements preintegrated_imu{preintegration_params, gm->imu_bias("imu")};

    // Preintegrate IMU measurements over sweep
    integrate_imu(preintegrated_imu, imu_s2s, msg->start_timestamp, msg->end_timestamp);
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
    const Eigen::Isometry3d s2s_pose_body =
            Eigen::Isometry3d(gm->pose("imu", -1).matrix()).inverse() * Eigen::Isometry3d(gm->pose("imu").matrix());
    // Convert to the lidar frame, T_{L_i-1}^{L_i}
    const eigen_ros::Pose s2s_pose_lidar{
            eigen_ext::change_relative_transform_frame(s2s_pose_body, body_frames.frame_to_body("lidar"))};
    ROS_WARN_ONCE("DESIGN DECISION: change output to odometry to pass velocity & covs to registration module?");

    // Publish imu estimated transform
    auto imu_transform = boost::make_shared<geometry_msgs::TransformStamped>();
    imu_transform->header.stamp = gm->timestamp("imu");  // Timestamp must be the child frame's for synchronisation
    imu_transform->header.frame_id = body_frames.frame_id("lidar") + "_i-1";  // i-1
    imu_transform->child_frame_id = body_frames.frame_id("lidar");            // i
    eigen_ros::to_ros(imu_transform->transform, s2s_pose_lidar);
    imu_transform_publisher.publish(imu_transform);

    // Add preintegration combined IMU factor (includes bias between factor) to graph
    if (imu_factors_enabled) {
        const int imu_key = gm->key("imu");
        gm->create_combined_imu_factor(imu_key, preintegrated_imu);
        ROS_INFO_STREAM("Created factor: X(" << imu_key - 1 << "), V(" << imu_key - 1 << "), B(" << imu_key - 1
                                             << ") => X(" << imu_key << "), V(" << imu_key << "), B(" << imu_key
                                             << ")");

        // Optimise if not waiting on other factors
        if (gm->minimum_key() == imu_key) {
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
        auto prior_velocity_noise =
                gtsam::noiseModel::Gaussian::Covariance(odometry.twist.linear_velocity_covariance());
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
        auto prior_imu_bias_noise =
                gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << prior_accel_bias_noise, prior_accel_bias_noise,
                        prior_accel_bias_noise, prior_gyro_bias_noise, prior_gyro_bias_noise, prior_gyro_bias_noise)
                                                            .finished());
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
    publish(key, optimise(key));
}

gtsam::ISAM2Result Optimisation::optimise(const int key) {
    // Optimise graph with iSAM2
    gtsam::ISAM2Result isam2_result;
    try {
        const ros::WallTime tic = ros::WallTime::now();
        isam2_result = gm->optimise(key);
        ROS_INFO_STREAM("Optimised and calculated estimate for key = "
                        << key << ", t = " << gm->timestamp(key) << " in " << (ros::WallTime::now() - tic).toSec()
                        << " seconds.\n#Reeliminated = " << isam2_result.getVariablesReeliminated()
                        << ", #Relinearized = " << isam2_result.getVariablesRelinearized() << ", #Cliques = "
                        << isam2_result.getCliques() << ", Factors Recalculated = " << isam2_result.factorsRecalculated
                        << (isam2_result.errorBefore && isam2_result.errorAfter
                                           ? "\nError Before = " + std::to_string(*isam2_result.errorBefore) +
                                                     ", Error After = " + std::to_string(*isam2_result.errorAfter)
                                           : ""));
    } catch (const gtsam::IndeterminantLinearSystemException& ex) {
        precrash_operations(ex);
        if (nh.param<bool>("debug/optimisation/on_crash/print", true)) {
            print_information_at_key(ex.nearbyVariable());
        }
        throw ex;
    } catch (const std::exception& ex) {
        precrash_operations(ex);
        throw ex;
    }
    return isam2_result;
}

void Optimisation::publish(const int key, const gtsam::ISAM2Result& isam2_result) {
    // Publish optimised pose
    auto optimised_odometry_msg = boost::make_shared<nav_msgs::Odometry>();
    optimised_odometry_msg->header.stamp = gm->timestamp(key);
    optimised_odometry_msg->header.frame_id = "map";
    optimised_odometry_msg->child_frame_id = body_frames.body_frame_id();
    auto pose = gm->pose(key);
    ROS_INFO_STREAM("Pose:\n" << pose.matrix());
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.position, pose.translation());
    eigen_ros::to_ros(optimised_odometry_msg->pose.pose.orientation, pose.rotation().toQuaternion());
    if (!stereo_factors_enabled) {
        auto pose_covariance = gm->pose_covariance(key);
        eigen_ros::to_ros(optimised_odometry_msg->pose.covariance, eigen_ext::reorder_covariance(pose_covariance, 3));
        ROS_INFO_STREAM("Pose Covariance (r, p, y, x, y, z):\n" << pose_covariance);
    } else {
        ROS_WARN_ONCE("Pose covariance computation skipped when stereo factors ENABLED - cannot run in real time");
    }
    eigen_ros::to_ros(optimised_odometry_msg->twist.twist.linear, gm->velocity(key));
    ROS_WARN_ONCE("TODO: add angular velocity from IMU odometry to optimised odometry");
    const Eigen::Matrix3d optimised_vel_covariance =
            imu_factors_enabled ? gm->velocity_covariance(key) : Eigen::Matrix3d::Zero();
    ROS_WARN_ONCE("TODO: add linear velocity covariance (optimised_vel_covariance) to optimised odometry");
    ROS_WARN_ONCE("TODO: add angular velocity covariance from IMU odometry to optimised odometry");
    optimised_odometry_publisher.publish(optimised_odometry_msg);
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
    auto global_path = boost::make_shared<nav_msgs::Path>(convert_to_path(*gm, key, body_frames.body_frame_id()));
    auto global_path_changes = boost::make_shared<nav_msgs::Path>(extract_changed_poses(*global_path, isam2_result));
    path_publisher.publish(global_path);
    path_changes_publisher.publish(global_path_changes);

    // Publish stereo points
    if (publish_stereo_points) {
        auto stereo_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl_conversions::toPCL(optimised_odometry_msg->header, stereo_points->header);
        to_pcl(gm->stereo_landmarks(), *stereo_points);
        stereo_points_publisher.publish(stereo_points);
    }
}

void Optimisation::precrash_operations(const std::exception& ex) {
    ROS_ERROR_STREAM("Optimisation failed with exception: " << ex.what());
    if (nh.param<bool>("debug/optimisation/on_crash/save", false)) {
        const std::string file_prefix{"serpent_crash"};
        gm->save(file_prefix);
        ROS_INFO_STREAM("Saved graph data with file prefix \'" << file_prefix << "\' to ~/.ros/");
    }
    if (nh.param<bool>("debug/optimisation/on_crash/print", true) &&
            nh.param<bool>("debug/optimisation/on_crash/print_options/verbose", false)) {
        const double min_error = nh.param<double>("debug/optimisation/on_crash/print_options/min_error", 0.0);
        ROS_INFO_STREAM("Printing all factors with error >= " << min_error);
        gm->print_errors(min_error);
    }
}

void Optimisation::print_information_at_key(const gtsam::Key key) {
    const gtsam::Symbol symbol{key};
    const gtsam::Value& value = gm->value(key);
    // Marginal covariance may not be available
    std::optional<Eigen::MatrixXd> covariance;
    try {
        covariance = gm->covariance(key);
    } catch (const std::exception& ex) {
    }
    std::stringstream ss;
    ss << "Information for: " << symbol << "\nMarginal Covariance:";
    if (covariance) {
        ss << "\n" << covariance.value();
    } else {
        ss << " n/a";
    }
    ss << "\nValue:";
    ROS_INFO_STREAM(ss.str());
    value.print();
    const gtsam::NonlinearFactorGraph connected_factors = gm->factors_for_key(key);
    ROS_INFO_STREAM("Connected factors:\n");
    int factor_counter{0};
    const gtsam::Values& values = gm->values();
    for (const auto& factor : connected_factors) {
        factor->print("Factor " + std::to_string(factor_counter++) + ": ");
        std::cerr << "error: " << factor->error(values) << "\n\n";
    }
}

void Optimisation::registration_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> guard{graph_mutex};

    // Integrate registration data
    add_registration_factor(msg);

    // Optimise and publish
    const int key = gm->key("reg");
    if (gm->minimum_key() == key) {
        // Optimise
        const auto optimisation_result = optimise(key);

        // Update values that weren't optimised
        if (!imu_factors_enabled) {
            update_velocity_from_transforms("reg");
        }

        // Publish
        publish(key, optimisation_result);
    }
}

void Optimisation::stereo_features_callback(const serpent::StereoFeatures::ConstPtr& features) {
    // Pose at (the next) stereo key must be set in order to create landmarks.
    if (!protected_sleep(graph_mutex, 0.01, false, true, [this]() { return !gm->has_pose("stereo", 1); })) {
        return;
    };

    // Integrate stereo data
    add_stereo_factors(features);

    // Optimise and publish
    if (gm->minimum_key() == gm->key("stereo") && gm->key("stereo") > 0) {
        optimise_and_publish(gm->key("stereo"));

        // Update velocity if no IMU factors
        if (!imu_factors_enabled) {
            update_velocity_from_transforms("stereo");
        }
    }

    graph_mutex.unlock();
}

void Optimisation::update_velocity_from_transforms(const std::string& named_key) {
    // Compute translation between poses in world frame
    const Eigen::Isometry3d previous_pose = eigen_gtsam::to_eigen<Eigen::Isometry3d>(gm->pose(named_key, -1));
    const Eigen::Isometry3d current_pose = eigen_gtsam::to_eigen<Eigen::Isometry3d>(gm->pose(named_key));
    const Eigen::Vector3d translation = current_pose.translation() - previous_pose.translation();

    // Comptue time between poses
    const double dt = gm->time_between(named_key, named_key, -1).toSec();

    // Compute and set linear velocity
    const Eigen::Vector3d linear_velocity = Eigen::Vector3d{translation.x(), translation.y(), translation.z()} / dt;
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
