#include "serpent/frontend.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>

#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <pointcloud_tools/pclpointcloud2_utilities.hpp>

#include "serpent/ImuArray.h"
#include "serpent/utilities.hpp"

namespace serpent {

StereoData change_timestamp(const StereoData& data, const ros::Time& timestamp) {
    // Changing timestamp requires a copy
    sensor_msgs::ImagePtr left_image = boost::make_shared<sensor_msgs::Image>(*data.left_image);
    left_image->header.stamp = timestamp;
    sensor_msgs::ImagePtr right_image = boost::make_shared<sensor_msgs::Image>(*data.right_image);
    right_image->header.stamp = timestamp;
    sensor_msgs::CameraInfoPtr left_info = boost::make_shared<sensor_msgs::CameraInfo>(*data.left_info);
    left_info->header.stamp = timestamp;
    sensor_msgs::CameraInfoPtr right_info = boost::make_shared<sensor_msgs::CameraInfo>(*data.right_info);
    right_info->header.stamp = timestamp;
    return StereoData{left_image, right_image, left_info, right_info};
}

Frontend::Frontend()
    : nh("serpent"),
      optimised_odometry_sync(10),
      it(nh),
      stereo_sync(10),
      body_frames("serpent"),
      last_preint_imu_timestamp(0.0),
      initialised(false) {
    // Publishers
    deskewed_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("frontend/deskewed_pointcloud", 1);
    imu_s2s_publisher = nh.advertise<serpent::ImuArray>("frontend/imu_s2s", 1);
    initial_odometry_publisher = nh.advertise<nav_msgs::Odometry>("frontend/initial_odometry", 1);
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("output/odometry", 1);
    pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("output/pose", 1);

    // Subscribers
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("input/imu", 5000, &Frontend::imu_callback, this);
    imu_biases_subscriber.subscribe(nh, "optimisation/imu_biases", 10);
    optimised_odometry_subscriber.subscribe(nh, "optimisation/odometry", 10);
    optimised_odometry_sync.connectInput(imu_biases_subscriber, optimised_odometry_subscriber);
    optimised_odometry_sync.registerCallback(boost::bind(&Frontend::optimised_odometry_callback, this, _1, _2));
    pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>("formatter/formatted_pointcloud", 1,
            &Frontend::pointcloud_callback, this);

    // Check base_link frame exists
    if (!body_frames.has_frame("base_link")) {
        throw std::runtime_error(
                "SERPENT requires base_link frame to be defined in order to publish transforms and "
                "poses from the map frame to it. A common use case is to set the body_frame_name to base_link, and "
                "then optionally to set the frame_id also (may differ from \"base_link\"). See README and "
                "body_frames.hpp documentation.");
    }

    // Map frame
    nh.param<std::string>("map_frame_id", map_frame_id, "map");

    // Motion distortion correction
    nh.param<bool>("mdc/translation", deskew_translation, true);
    nh.param<bool>("mdc/rotation", deskew_rotation, true);
    nh.param<bool>("stereo_tracking/only_graph_frames", only_graph_frames, false);

    // Stereo
    nh.param<bool>("optimisation/factors/stereo", stereo_enabled, true);
    if (stereo_enabled) {
        // MDC must be enabled
        if (!deskew_translation || !deskew_rotation) {
            throw std::runtime_error("Motion distortion correction (mdc) must be fully enabled with stereo factors "
                                     "enabled.");
        }

        // Publishers
        left_image_publisher = it.advertise("stereo/left/image", 1);
        right_image_publisher = it.advertise("stereo/right/image", 1);
        left_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/left/camera_info", 1);
        right_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/right/camera_info", 1);

        // Subscribers
        left_image_subcriber.subscribe(nh, "input/stereo/left/image", 10);
        right_image_subcriber.subscribe(nh, "input/stereo/right/image", 10);
        left_info_subcriber.subscribe(nh, "input/stereo/left/camera_info", 10);
        right_info_subcriber.subscribe(nh, "input/stereo/right/camera_info", 10);
        stereo_sync.connectInput(left_image_subcriber, right_image_subcriber, left_info_subcriber,
                right_info_subcriber);
        stereo_sync.registerCallback(boost::bind(&Frontend::stereo_callback, this, _1, _2, _3, _4));
    }

    // Preintegration parameters
    nh.param<bool>("imu/noise/overwrite", overwrite_imu_covariance, false);
    preintegration_params = gtsam::PreintegrationCombinedParams::MakeSharedU(nh.param<double>("gravity", 9.81));
    ROS_WARN_ONCE("DESIGN DECISION: gravity from initialisation procedure?");
    preintegration_params->setIntegrationCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/integration", 1.0e-3), 2.0));
#if GTSAM_VERSION_NUMERIC >= 40200
    preintegration_params->setBiasAccOmegaInit(Eigen::Matrix<double, 6, 6>::Identity() *
                                               std::pow(nh.param<double>("imu/noise/integration_bias", 1.0e-3), 2.0));
#else
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
                                              std::pow(nh.param<double>("imu/noise/integration_bias", 1.0e-3), 2.0));
#endif
    preintegration_params->setBiasAccCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope_bias", 1.0e-3), 2.0));
    if (overwrite_imu_covariance) {
        accelerometer_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer", 1.0e-3), 2.0);
        gyroscope_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope", 1.0e-3), 2.0);
        ROS_INFO_STREAM("IMU accelerometer and gyroscope covariances will be overwritten.");
        ROS_INFO_STREAM("Accelerometer covariance:\n" << accelerometer_covariance);
        ROS_INFO_STREAM("Gyroscope covariance:\n" << gyroscope_covariance);
        update_preintegration_params(*preintegration_params, accelerometer_covariance, gyroscope_covariance);
    }
    // pose of the sensor in the body frame
    const gtsam::Pose3 body_to_imu = eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame("imu"));
    preintegration_params->setBodyPSensor(body_to_imu);

    // Barometer
    nh.param<bool>("optimisation/factors/barometer", barometer_enabled, true);
    if (barometer_enabled) {
        barometer_publisher = nh.advertise<sensor_msgs::FluidPressure>("frontend/barometer", 1);
        barometer_subscriber =
                nh.subscribe<sensor_msgs::FluidPressure>("input/barometer", 100, &Frontend::barometer_callback, this);
    }
    nh.param<bool>("barometer/noise/overwrite", overwrite_barometer_variance, false);
    if (overwrite_barometer_variance) {
        barometer_variance = std::pow(nh.param<double>("barometer/noise/barometer", 0.01), 2.0);
        if (barometer_variance <= 0.0) {
            throw std::runtime_error("Barometer noise should be >= zero with overwrite set to true.");
        }
    }
}

void Frontend::barometer_callback(const sensor_msgs::FluidPressure::ConstPtr& pressure) {
    std::lock_guard<std::mutex> guard{barometer_data_mutex};
    if (overwrite_barometer_variance) {
        auto pressure_ = boost::make_shared<sensor_msgs::FluidPressure>(*pressure);
        pressure_->variance = barometer_variance;
        barometer_buffer.push_back(pressure_);
    } else {
        barometer_buffer.push_back(pressure);
    }
    barometer_try_publish();
}

void Frontend::barometer_try_publish() {
    if (!barometer_timestamp_buffer.empty()) {
        // Special case (first measurement) where barometer data might not preceded first state timestamp
        if (barometer_buffer.front()->header.stamp > barometer_timestamp_buffer.front()) {
            ROS_WARN_STREAM("No barometer measurement prior to new state timestamp. Approximating with measurement "
                            << (barometer_buffer.front()->header.stamp - barometer_timestamp_buffer.front()).toSec()
                            << "s ahead of state timestamp.");
            auto pressure = boost::make_shared<sensor_msgs::FluidPressure>(*barometer_buffer.front());
            pressure->header.stamp = barometer_timestamp_buffer.front();
            barometer_publisher.publish(pressure);
            barometer_timestamp_buffer.pop_front();
        } else if (barometer_buffer.size() >= 2 &&
                   barometer_buffer.back()->header.stamp > barometer_timestamp_buffer.front()) {
            // Remove old barometer messages until front is the first measurement before the state time
            while (barometer_buffer.at(1)->header.stamp < barometer_timestamp_buffer.front()) {
                barometer_buffer.pop_front();
            }

            // Interpolate and publish
            barometer_publisher.publish(interpolate_pressure(*barometer_buffer.front(), *barometer_buffer.at(1),
                    barometer_timestamp_buffer.front()));
            barometer_timestamp_buffer.pop_front();
        }
    }
}

Eigen::Quaterniond Frontend::body_frame_orientation(const eigen_ros::Imu& imu) const {
    Eigen::Quaterniond body_orientation;
    if (imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0))) {
        body_orientation = Eigen::Quaterniond::Identity();
    } else {
        // Compute T_R^B = T_R^I * T_I^B where R = imu_reference_frame. If R = NWU our computation is done.
        body_orientation = imu.orientation * Eigen::Quaterniond(body_frames.frame_to_body("imu").rotation());
        const std::string imu_reference_frame = nh.param<std::string>("imu/reference_frame", "NED");
        if (imu_reference_frame == "NED") {
            // Since body orientation is relative NWU (map), compute T_NWU^B = T_NWU^NED * T_NED^B
            body_orientation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * body_orientation;
        }
    }
    return body_orientation;
}

void Frontend::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert IMU to pointcloud frame using extrinsics
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    if (overwrite_imu_covariance) {
        imu.linear_acceleration_covariance = accelerometer_covariance;
        imu.angular_velocity_covariance = gyroscope_covariance;
    }

    // Save transformed IMU message to buffer for mdc
    std::lock_guard<std::mutex> guard{preintegration_mutex};
    imu_data_mutex.lock();
    double dt = (imu.timestamp - last_preint_imu_timestamp).toSec();  // on 1st iteration invalid but not used
    imu_buffer.push_back(imu);
    imu_data_mutex.unlock();
    last_preint_imu_timestamp = imu.timestamp;

    if (preintegrated_imu) {
        // Integrate current IMU measurement into running integration to obtain current incremental transform
        preintegrated_imu->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);

        // Calculate current state from previous state
        const gtsam::NavState state = preintegrated_imu->predict(world_state, imu_biases);
        /*
        // TODO: Combine optimised odometry covariances (in world_odometry) with state_covariance from pre-integration
        // NOTE: optimised odometry covariance is disabled for stereo factors
        const gtsam::Matrix15 state_covariance = preintegrated_imu->preintMeasCov(); // rot, pos, vel, accel, gyro
        const Eigen::Matrix<double, 6, 6> pose_covariance = state_covariance.block<6, 6>(0, 0);
        const Eigen::Matrix3d linear_velocity_covariance = state_covariance.block<3, 3>(6, 6);
        Eigen::Matrix<double, 6, 6> twist_covariance;
        twist_covariance << imu.angular_velocity_covariance, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                linear_velocity_covariance;
        */
        ROS_WARN_ONCE("TODO FIX: angular velocity must be converted from IMU frame to body frame - is this even "
                      "possible? A rotation may be a good approximation.");
        const Eigen::Vector3d angular_velocity =
                body_frames.body_to_frame("imu").rotation() * (imu.angular_velocity + imu_biases.gyroscope());

        // Publish current state as odometry output
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = map_frame_id;
        odometry->child_frame_id = body_frames.body_frame_id();
        eigen_ros::to_ros(odometry->pose.pose.position, state.position());
        eigen_ros::to_ros(odometry->pose.pose.orientation, state.attitude().toQuaternion());
        // eigen_ros::to_ros(odometry->pose.covariance, eigen_ext::reorder_covariance(pose_covariance, 3));
        eigen_ros::to_ros(odometry->twist.twist.linear, state.velocity());
        eigen_ros::to_ros(odometry->twist.twist.angular, angular_velocity);
        // eigen_ros::to_ros(odometry->twist.covariance, eigen_ext::reorder_covariance(twist_covariance, 3));
        ROS_WARN_ONCE("TODO FIX: IMU-rate odometry may not be valid");
        odometry_publisher.publish(odometry);
    }
}

eigen_ros::Imu Frontend::interpolated_imu(const ros::Time interp_timestamp) const {
    std::lock_guard<std::mutex> guard{imu_data_mutex};
    if (imu_buffer.size() < 2 || imu_buffer.front().timestamp > interp_timestamp ||
            imu_buffer.back().timestamp < interp_timestamp) {
        throw std::runtime_error("No IMU messages around interpolated timestamp.");
    }
    std::size_t i{1};  // Index of imu message after deskew timestamp (which is guaranteed to exist).
    while (imu_buffer.at(i).timestamp < interp_timestamp) ++i;
    return eigen_ros::interpolate(imu_buffer.at(i - 1), imu_buffer.at(i), interp_timestamp);
}

void Frontend::optimised_odometry_callback(const serpent::ImuBiases::ConstPtr& imu_biases_msg,
        const nav_msgs::Odometry::ConstPtr& optimised_odometry_msg) {
    std::lock_guard<std::mutex> preintegration_guard{preintegration_mutex};

    // Save optimised odometry
    world_odometry = eigen_ros::from_ros<eigen_ros::Odometry>(*optimised_odometry_msg);
    world_state = gtsam::NavState(gtsam::Rot3(world_odometry.pose.orientation), world_odometry.pose.position,
            world_odometry.twist.linear);

    // Compute T_W^{BL} TF at t_i-1 for output (note that the odometry pose is T_W^B)
    try {
        // Avoid transformation of pose and covariance if we can.
        eigen_ros::PoseStamped map_to_base_link = world_odometry.pose_stamped();
        if (body_frames.body_frame() != "base_link") {
            // Compute transform from map to base_link (including covariance): T_W^{BL} = T_W^B * T_B^{BL}
            map_to_base_link = eigen_ros::apply_transform(map_to_base_link,
                    body_frames.body_to_frame("base_link", world_odometry.timestamp));
        }

        // Broadcast T_W^{BL} TF
        auto map_to_base_link_tf = eigen_ros::to_ros<geometry_msgs::TransformStamped>(map_to_base_link);
        map_to_base_link_tf.header.frame_id = optimised_odometry_msg->header.frame_id;
        map_to_base_link_tf.child_frame_id = body_frames.frame_id("base_link");
        tf_broadcaster.sendTransform(map_to_base_link_tf);

        // Publish T_W^{BL} pose with covariance
        auto map_to_base_link_pose = eigen_ros::to_ros<geometry_msgs::PoseWithCovarianceStamped>(map_to_base_link);
        map_to_base_link_pose.header.frame_id = optimised_odometry_msg->header.frame_id;
        pose_publisher.publish(map_to_base_link_pose);
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM("Failed to publish map_frame -> base_link_frame TF or pose. Exception: " << ex.what());
    }

    // Save biases
    from_ros(*imu_biases_msg, imu_biases);
    imu_bias_timestamp = imu_biases_msg->header.stamp;
    ROS_DEBUG_STREAM("Updated IMU bias at t = " << imu_bias_timestamp);

    // Update the preintegration parameters according to the next IMU message (after the imu bias timestamp)
    const eigen_ros::Imu imu_bias_imu = interpolated_imu(imu_bias_timestamp);
    ROS_WARN_ONCE("DESIGN DECISION: gravity from IMU measurement?");
    update_preintegration_params(*preintegration_params, imu_bias_imu.linear_acceleration_covariance,
            imu_bias_imu.angular_velocity_covariance);
    ROS_DEBUG_STREAM("Update preintegration parameters using imu message at t = " << imu_bias_imu.timestamp);

    // Create new IMU integrator with new biases for IMU-rate odometry
    preintegrated_imu = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_biases);
    last_preint_imu_timestamp = imu_bias_timestamp;

    // Integrate IMU messages after last_preint_imu_timestamp
    std::lock_guard<std::mutex> imu_data_guard{imu_data_mutex};
    for (const auto& imu : imu_buffer) {
        if (imu.timestamp > last_preint_imu_timestamp) {
            const double dt = (imu.timestamp - last_preint_imu_timestamp).toSec();
            last_preint_imu_timestamp = imu.timestamp;
            preintegrated_imu->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
        }
    }
}

void Frontend::pointcloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    // Error handling
    if (pct::empty(*msg)) {
        throw std::runtime_error("Pointcloud seq=" + std::to_string(msg->header.seq) +
                                 " was empty. Handling of empty pointclouds not yet supported");
    }

    // Pointcloud start time
    const ros::Time pointcloud_start = pcl_conversions::fromPCL(msg->header.stamp);
    ROS_DEBUG_STREAM("Received pointcloud (seq=" << msg->header.seq << ", " << pct::size_points(*msg) << " pts, "
                                                 << pointcloud_start << ")");

    // Process the information between the previous and current pointcloud
    if (previous_pointcloud) {
        // Previous pointcloud start time
        const ros::Time previous_pointcloud_start = pcl_conversions::fromPCL(previous_pointcloud->header.stamp);
        ROS_INFO_STREAM("Processing data between previous (t = " << previous_pointcloud_start << ") and current (t = "
                                                                 << pointcloud_start << ") pointclouds.");

        // Set new state, depending on stereo data. If stereo disabled or no stereo frame exista, use the start time.
        ros::Time new_state_time = previous_pointcloud_start;
        if (stereo_enabled) {
            // Remove old stereo data and wait for stereo data after previous_pointcloud_start (may arrive late)
            stereo_data_mutex.lock();
            ROS_INFO_COND(stereo_buffer.empty() || stereo_buffer.back().timestamp() < previous_pointcloud_start,
                    "Waiting for stereo data to arrive before selecting new optimisation state.");
            if (!protected_sleep(stereo_data_mutex, 0.01, true, true, [this, previous_pointcloud_start]() {
                    while (!stereo_buffer.empty() && stereo_buffer.front().timestamp() < previous_pointcloud_start) {
                        stereo_buffer.pop_front();
                    }
                    return stereo_buffer.empty();
                })) {
                return;
            };

            // Check for stereo data between the point clouds.
            if (stereo_buffer.front().timestamp() < pointcloud_start) {
                // Update the deskew time.
                new_state_time = stereo_buffer.front().timestamp();
                // Publish stereo data if it hasn't been already.
                if (only_graph_frames) {
                    publish_stereo_data(stereo_buffer.front());
                }
                // Remove stereo data from queue.
                stereo_buffer.pop_front();
            }
            stereo_data_mutex.unlock();
        }
        ROS_INFO_STREAM("New state time set to " << new_state_time << ".");

        // Initialised: publish the IMU messages between the deskew times of the pointclouds.
        // Not initialised: construct and publish the initial odometry at the deskew timestamp.
        if (initialised) {
            // Accumulate and publish the IMU measurements required for registration of the previous two pointclouds.
            // This spans from the previous to the current state time. Remove any IMU measurements older
            // than the first of these.
            std::lock_guard<std::mutex> guard{imu_data_mutex};
            auto imu_s2s = boost::make_shared<serpent::ImuArray>();
            imu_s2s->start_timestamp = previous_state_time;
            imu_s2s->end_timestamp = new_state_time;
            if (imu_buffer.front().timestamp > previous_state_time) {
                throw std::runtime_error("IMU measurement before the previous deskew timestamp was required.");
            }
            for (auto it = imu_buffer.cbegin(); it != imu_buffer.cend(); ++it) {
                // Add the IMU measurement if it is between the pointcloud timestamps or is the first measurement before
                // the previous pointcloud start time (and next measurement isn't exactly the start time).
                if (it->timestamp < new_state_time &&
                        (it->timestamp > previous_state_time || std::next(it)->timestamp > previous_state_time)) {
                    imu_s2s->measurements.push_back(eigen_ros::to_ros<sensor_msgs::Imu>(*it));
                }
            }
            delete_old_measurements(imu_s2s->measurements.front().header.stamp, imu_buffer);
            imu_s2s_publisher.publish(imu_s2s);
            ROS_DEBUG_STREAM(
                    "Published IMU measurements from " << imu_s2s->start_timestamp << " to " << imu_s2s->end_timestamp);
        } else {
            // Wait for IMU message after deskew timestamp (for safety, as should already be received).
            if (!protected_sleep(preintegration_mutex, 0.01, false, false,
                        [this, new_state_time]() { return last_preint_imu_timestamp < new_state_time; })) {
                return;
            };

            // Compute the interpolated IMU message at the deskew timestamp.
            const eigen_ros::Imu deskew_imu = interpolated_imu(new_state_time);

            // Compute initial state using user-defined position & yaw, and gravity-defined roll and pitch
            const Eigen::Matrix3d position_covariance =
                    Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/position", 1.0e-3), 2.0);
            const Eigen::Matrix3d rotation_covariance =
                    Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/rotation", 1.0e-3), 2.0);
            const Eigen::Quaterniond body_orientation = body_frame_orientation(deskew_imu);
            const eigen_ros::Pose pose{
                    Eigen::Vector3d(nh.param<double>("pose/position/x", 0.0), nh.param<double>("pose/position/y", 0.0),
                            nh.param<double>("pose/position/z", 0.0)),
                    body_orientation, position_covariance, rotation_covariance};
            ROS_WARN_ONCE("TODO FIX: initial pose based on orientation but should be based on gravity. Can't use "
                          "orientation for 6-axis IMU");
            const Eigen::Matrix3d linear_twist_covariance =
                    Eigen::Matrix3d::Identity() *
                    std::pow(nh.param<double>("prior_noise/linear_velocity", 1.0e-3), 2.0);
            const Eigen::Vector3d linear_velocity = Eigen::Vector3d{nh.param<double>("velocity/linear/x", 0.0),
                    nh.param<double>("velocity/linear/y", 0.0), nh.param<double>("velocity/linear/z", 0.0)};
            ROS_WARN_ONCE("TODO FIX: angular velocity and cov must be converted from IMU frame to body frame");
            const eigen_ros::Twist twist{linear_velocity, deskew_imu.angular_velocity, linear_twist_covariance,
                    deskew_imu.angular_velocity_covariance};
            const eigen_ros::Odometry init_odometry{pose, twist, new_state_time, map_frame_id,
                    body_frames.body_frame_id()};

            // Publish initial state
            auto odometry_msg = boost::make_shared<nav_msgs::Odometry>();
            eigen_ros::to_ros(*odometry_msg, init_odometry);
            initial_odometry_publisher.publish(odometry_msg);
            ROS_INFO_STREAM("Sent initial odometry message (t = " << new_state_time << ").");

            // Set previous deskew timestamp, because deskewing the first and second pointclouds use the same imu bias.
            previous_state_time = new_state_time;
        }

        // Compute point cloud sweep end time
        const ros::Duration sweep_duration{
                pct::has_field(*previous_pointcloud, "t") ? pct::max<float>(*previous_pointcloud, "t") : 0};
        const ros::Time previous_pointcloud_end = previous_pointcloud_start + sweep_duration;

        // Wait for previous imu_biases (& world_state) and IMU message past end of sweep and start of new point cloud.
        // Then imu_biases, world_state, world_odometry and preintegration_params can be used without mutex locking
        // because the optimised odometry callback cannot trigger until after the deskewed pointcloud is published.
        if (!protected_sleep(preintegration_mutex, 0.01, false, false, [this, previous_pointcloud_end]() {
                return imu_bias_timestamp != previous_state_time || last_preint_imu_timestamp < previous_pointcloud_end;
            })) {
            return;
        };

        // Create new IMU integrator for deskewing with updated parameters and biases
        gtsam::PreintegratedCombinedMeasurements preintegrated_imu_skew{preintegration_params, imu_biases};

        // Compute previous pointcloud start state (state at previous_pointcloud_start) from world_state (in body frame)
        imu_data_mutex.lock();
        gtsam::NavState previous_pointcloud_start_state;
        if (!initialised) {
            ROS_WARN_ONCE("TODO FIX: The state for the first pointcloud is approximated as the initial state. This is "
                          "valid if the initial state timestamp == pointcloud timestamp (e.g. without stereo or with "
                          "perfectly synchronised stereo), and a valid approximation if motion is negligible between "
                          "the timestamps. The correct fix is to perform backwards IMU integration from the initial "
                          "state to the pointcloud timestamp, however this is difficult.");
            // Generate an approximate start state
            previous_pointcloud_start_state = world_state;
        } else {
            // Integrate IMU messages from previous optimised state to previous pointcloud timestamp
            integrate_imu(preintegrated_imu_skew, imu_buffer, previous_state_time, previous_pointcloud_start);

            // Predict state at pointcloud start time (body frame)
            previous_pointcloud_start_state = preintegrated_imu_skew.predict(world_state, imu_biases);

            // Reset Integration for deskewing
            preintegrated_imu_skew.resetIntegration();
        }

        // Integrate over pointcloud sweep
        integrate_imu(preintegrated_imu_skew, imu_buffer, previous_pointcloud_start, previous_pointcloud_end);
        imu_data_mutex.unlock();

        // Generate transform from preintegration, which is in the body frame.
        // T_{B_{i,s}}^{B_{i,e}} = T_{B_{i,s}}^{W} * T_{W}^{B_{i,e}}
        const gtsam::NavState previous_pointcloud_end_state =
                preintegrated_imu_skew.predict(previous_pointcloud_start_state, imu_biases);
        Eigen::Isometry3d skew_transform =
                eigen_gtsam::to_eigen<Eigen::Isometry3d>(previous_pointcloud_start_state.pose()).inverse() *
                eigen_gtsam::to_eigen<Eigen::Isometry3d>(previous_pointcloud_end_state.pose());

        // Transform skew_transform from body frame into lidar frame
        skew_transform = eigen_ext::change_relative_transform_frame(skew_transform, body_frames.body_to_frame("lidar"));

        // Deskew configuration
        skew_transform = eigen_ext::to_transform(
                (deskew_translation ? skew_transform.translation() : Eigen::Vector3d{0.0, 0.0, 0.0}),
                (deskew_rotation ? skew_transform.rotation() : Eigen::Matrix3d::Identity()));

        // Deskew pointcloud
        pcl::PCLPointCloud2::Ptr deskewed_pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
        const ros::WallTime tic = ros::WallTime::now();
        pct::deskew(skew_transform, sweep_duration.toSec(), pcl_conversions::toPCL(new_state_time),
                *previous_pointcloud, *deskewed_pointcloud);
        ROS_INFO_STREAM(
                "Deskewed pointcloud to " << new_state_time << " in " << (ros::WallTime::now() - tic).toSec() << " s.");

        // Publish deskewed pointcloud
        deskewed_pointcloud_publisher.publish(deskewed_pointcloud);

        // Save timestamp for barometer interpolation
        if (barometer_enabled) {
            std::lock_guard<std::mutex> guard{barometer_data_mutex};
            barometer_timestamp_buffer.push_back(new_state_time);
            barometer_try_publish();
        }

        // Update previous_state_time
        previous_state_time = new_state_time;

        // Initialisation complete
        initialised = true;
    }

    // Save pointcloud for next callback
    previous_pointcloud = msg;
}

void Frontend::publish_stereo_data(const StereoData& data, const ros::Time& timestamp) {
    if (timestamp == ros::Time() || data.timestamp() == timestamp) {
        left_image_publisher.publish(data.left_image);
        right_image_publisher.publish(data.right_image);
        left_info_publisher.publish(data.left_info);
        right_info_publisher.publish(data.right_info);
        ROS_DEBUG_STREAM("Re-publishing stereo data at its original timestamp");
    } else {
        // Changing timestamp requires a copy
        sensor_msgs::ImagePtr left_image = boost::make_shared<sensor_msgs::Image>(*data.left_image);
        left_image->header.stamp = timestamp;
        sensor_msgs::ImagePtr right_image = boost::make_shared<sensor_msgs::Image>(*data.right_image);
        right_image->header.stamp = timestamp;
        sensor_msgs::CameraInfoPtr left_info = boost::make_shared<sensor_msgs::CameraInfo>(*data.left_info);
        left_info->header.stamp = timestamp;
        sensor_msgs::CameraInfoPtr right_info = boost::make_shared<sensor_msgs::CameraInfo>(*data.right_info);
        right_info->header.stamp = timestamp;
        left_image_publisher.publish(left_image);
        right_image_publisher.publish(right_image);
        left_info_publisher.publish(left_info);
        right_info_publisher.publish(right_info);
        ROS_DEBUG_STREAM("Re-publishing stereo data with new timestamp = "
                         << timestamp << " (originally " << data.left_image->header.stamp
                         << ", diff = " << (data.left_image->header.stamp - timestamp).toSec() << " s)");
    }
}

void Frontend::stereo_callback(const sensor_msgs::ImageConstPtr& left_image,
        const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_info,
        const sensor_msgs::CameraInfoConstPtr& right_info) {
    std::lock_guard<std::mutex> guard{stereo_data_mutex};
    // Since PCL timestamps are accurate to the microsecond, the stereo timestamps must also be modified to ensure sync
    const ros::Time timestamp = pcl_conversions::fromPCL(pcl_conversions::toPCL(left_image->header.stamp));
    // Add stereo data to queue
    ROS_WARN_ONCE("TODO: Fix inefficiency where stereo data is always copied to change timestamp, even when "
                  "only_graph_frames is true.");
    stereo_buffer.push_back(change_timestamp(StereoData{left_image, right_image, left_info, right_info}, timestamp));
    ROS_DEBUG_STREAM("Saved stereo image (t = " << stereo_buffer.back().timestamp() << ").");
    // If all stereo frames will be republished, republish immediately.
    if (!only_graph_frames) {
        publish_stereo_data(stereo_buffer.back());
    }
}
}
