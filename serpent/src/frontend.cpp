#include "serpent/frontend.hpp"

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

Frontend::Frontend()
    : nh("~"),
      optimised_odometry_sync(10),
      it(nh),
      stereo_sync(10),
      last_preint_imu_timestamp(0.0),
      initialised(false),
      publish_next_stereo(false) {
    // Publishers
    deskewed_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("frontend/deskewed_pointcloud", 1);
    imu_s2s_publisher = nh.advertise<serpent::ImuArray>("frontend/imu_s2s", 1);
    initial_odometry_publisher = nh.advertise<nav_msgs::Odometry>("frontend/initial_odometry", 1);
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("output/odometry", 1);

    // Subscribers
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("input/imu", 1000, &Frontend::imu_callback, this);
    imu_biases_subscriber.subscribe(nh, "optimisation/imu_biases", 10);
    optimised_odometry_subscriber.subscribe(nh, "optimisation/odometry", 10);
    optimised_odometry_sync.connectInput(imu_biases_subscriber, optimised_odometry_subscriber);
    optimised_odometry_sync.registerCallback(boost::bind(&Frontend::optimised_odometry_callback, this, _1, _2));
    pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>("formatter/formatted_pointcloud", 100,
            &Frontend::pointcloud_callback, this);

    // Motion distortion correction
    nh.param<bool>("mdc/translation", deskew_translation, false);
    nh.param<bool>("mdc/rotation", deskew_rotation, false);

    // Stereo data
    nh.param<bool>("optimisation/factors/stereo", stereo_enabled, true);
    if (stereo_enabled) {
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
    preintegration_params->setBiasAccOmegaInt(Eigen::Matrix<double, 6, 6>::Identity() *
                                              std::pow(nh.param<double>("imu/noise/integration_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasAccCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer_bias", 1.0e-3), 2.0));
    preintegration_params->setBiasOmegaCovariance(
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope_bias", 1.0e-3), 2.0));
    if (overwrite_imu_covariance) {
        overwrite_accelerometer_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/accelerometer", 1.0e-3), 2.0);
        overwrite_gyroscope_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("imu/noise/gyroscope", 1.0e-3), 2.0);
        ROS_INFO_STREAM("IMU accelerometer and gyroscope covariances will be overwritten.");
        ROS_INFO_STREAM("Accelerometer covariance:\n" << overwrite_accelerometer_covariance);
        ROS_INFO_STREAM("Gyroscope covariance:\n" << overwrite_gyroscope_covariance);
        update_preintegration_params(*preintegration_params, overwrite_accelerometer_covariance,
                overwrite_gyroscope_covariance);
    }
    // pose of the sensor in the body frame
    const gtsam::Pose3 body_to_imu = eigen_gtsam::to_gtsam<gtsam::Pose3>(body_frames.body_to_frame("imu"));
    preintegration_params->setBodyPSensor(body_to_imu);
}

void Frontend::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert IMU to pointcloud frame using extrinsics
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    if (overwrite_imu_covariance) {
        imu.linear_acceleration_covariance = overwrite_accelerometer_covariance;
        imu.angular_velocity_covariance = overwrite_gyroscope_covariance;
    }

    // Save transformed IMU message to buffer for mdc
    std::lock_guard<std::mutex> guard(optimised_odometry_mutex);
    imu_mutex.lock();
    double dt = (imu.timestamp - last_preint_imu_timestamp).toSec();  // invalid but not used on 1st iteration
    last_preint_imu_timestamp = imu.timestamp;
    imu_buffer.push_back(imu);
    imu_mutex.unlock();

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
        ROS_WARN_ONCE(
                "TODO FIX: angular velocity must be converted from IMU frame to body frame - is this even possible? A "
                "rotation may be a good approximation.");
        const Eigen::Vector3d angular_velocity =
                body_frames.body_to_frame("imu").rotation() * (imu.angular_velocity + imu_biases.gyroscope());

        // Publish current state as odometry output
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = "map";
        odometry->child_frame_id = body_frames.body_frame_id();
        eigen_ros::to_ros(odometry->pose.pose.position, state.position());
        eigen_ros::to_ros(odometry->pose.pose.orientation, state.attitude().toQuaternion());
        // eigen_ros::to_ros(odometry->pose.covariance, eigen_ext::reorder_covariance(pose_covariance, 3));
        eigen_ros::to_ros(odometry->twist.twist.linear, state.velocity());
        eigen_ros::to_ros(odometry->twist.twist.angular, angular_velocity);
        // eigen_ros::to_ros(odometry->twist.covariance, eigen_ext::reorder_covariance(twist_covariance, 3));
        ROS_WARN_ONCE("TODO FIX: IMU-rate odometry is not valid");
        odometry_publisher.publish(odometry);
    }
}

void Frontend::optimised_odometry_callback(const serpent::ImuBiases::ConstPtr& imu_biases_msg,
        const nav_msgs::Odometry::ConstPtr& optimised_odometry_msg) {
    std::lock_guard<std::mutex> guard{optimised_odometry_mutex};

    // Save optimised odometry
    world_odometry = eigen_ros::from_ros<eigen_ros::Odometry>(*optimised_odometry_msg);
    world_state = gtsam::NavState(gtsam::Rot3(world_odometry.pose.orientation), world_odometry.pose.position,
            world_odometry.twist.linear);

    // Publish map to body TF at t_i-1
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = optimised_odometry_msg->header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = body_frames.body_frame_id();
    tf.transform.translation.x = optimised_odometry_msg->pose.pose.position.x;
    tf.transform.translation.y = optimised_odometry_msg->pose.pose.position.y;
    tf.transform.translation.z = optimised_odometry_msg->pose.pose.position.z;
    tf.transform.rotation = optimised_odometry_msg->pose.pose.orientation;
    tf_broadcaster.sendTransform(tf);

    // Save biases
    from_ros(*imu_biases_msg, imu_biases);
    imu_bias_timestamp = imu_biases_msg->header.stamp;
    ROS_INFO_STREAM("Updated IMU bias at t = " << imu_bias_timestamp);

    // Create new IMU integrator with new biases for IMU-rate odometry
    preintegrated_imu = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(preintegration_params, imu_biases);
    last_preint_imu_timestamp = imu_biases_msg->header.stamp;

    // Integrate IMU messages after last_preint_imu_timestamp
    for (const auto& imu : imu_buffer) {
        if (imu.timestamp > last_preint_imu_timestamp) {
            const double dt = (imu.timestamp - last_preint_imu_timestamp).toSec();
            last_preint_imu_timestamp = imu.timestamp;
            preintegrated_imu->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
        }
    }
}

void Frontend::pointcloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    // Save pointcloud start time
    const ros::Time pointcloud_start = pcl_conversions::fromPCL(msg->header.stamp);
    ROS_INFO_STREAM("Received pointcloud seq=" << msg->header.seq << " (" << pct::size_points(*msg)
                                               << " pts) with timestamp " << pointcloud_start);
    if (pct::empty(*msg)) {
        throw std::runtime_error("Handling of empty pointclouds not yet supported");
    }

    // Publish first stereo data after pointcloud timestamp
    if (stereo_enabled) {
        std::lock_guard<std::mutex> guard{stereo_mutex};
        // Remove old data
        while (!stereo_data.empty() && stereo_data.front().timestamp() < pointcloud_start) {
            stereo_data.pop_front();
        }
        // Publish stereo data or inform stereo thread to publish next received data
        if (stereo_data.empty()) {
            publish_next_stereo = true;
            publish_next_stereo_timestamp = pointcloud_start;
        } else {
            publish_stereo_data(stereo_data.front(), pointcloud_start);
            stereo_data.pop_front();
        }
    }

    // Wait until previous imu_biases received (before sending publishing IMU S2S)
    ROS_INFO_STREAM("Waiting for previous bias at " << previous_pointcloud_start);
    if (!protected_sleep(optimised_odometry_mutex, 0.01, false, true,
                [this]() { return imu_bias_timestamp != previous_pointcloud_start; })) {
        return;
    };
    const gtsam::imuBias::ConstantBias previous_imu_biases = imu_biases;
    ROS_INFO_STREAM("Acquired previous bias at " << imu_bias_timestamp);
    optimised_odometry_mutex.unlock();

    // Sleep until IMU message after scan start time
    ROS_INFO_STREAM("Waiting for start time IMU message past " << pointcloud_start);
    if (!protected_sleep(imu_mutex, 0.01, false, true,
                [this, pointcloud_start]() { return last_preint_imu_timestamp < pointcloud_start; })) {
        return;
    };
    if (imu_buffer.back().timestamp < pointcloud_start) {
        throw std::runtime_error("IMU buffer does not contain a recent enough IMU message. Something went wrong.");
    }
    ROS_INFO_STREAM("Acquired start time IMU message at " << last_preint_imu_timestamp);

    if (initialised) {
        // Accumulate and publish IMU measurements from previous to current scan.
        auto imu_s2s = boost::make_shared<serpent::ImuArray>();
        imu_s2s->start_timestamp = previous_pointcloud_start;
        imu_s2s->end_timestamp = pointcloud_start;
        imu_s2s->measurements = old_messages_to_ros(pointcloud_start, imu_buffer);
        imu_s2s_publisher.publish(imu_s2s);
    }

    // Get imu message after pointcloud start time
    eigen_ros::Imu pc_start_imu;
    for (const auto& imu : imu_buffer) {
        if (imu.timestamp >= pointcloud_start) {
            pc_start_imu = imu;
        }
    }
    imu_mutex.unlock();

    // Update preintegration parameters, with IMU noise at scan start time
    optimised_odometry_mutex.lock();
    ROS_WARN_ONCE("DESIGN DECISION: gravity from IMU measurements?");
    update_preintegration_params(*preintegration_params, pc_start_imu.linear_acceleration_covariance,
            pc_start_imu.angular_velocity_covariance);
    optimised_odometry_mutex.unlock();

    // If first time, perform initialisation procedure.
    Eigen::Isometry3d skew_transform;
    if (!initialised) {
        // Compute initial state using user-defined position & yaw, and gravity-defined roll and pitch
        const Eigen::Matrix3d position_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/position", 1.0e-3), 2.0);
        const Eigen::Matrix3d rotation_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/rotation", 1.0e-3), 2.0);
        Eigen::Quaterniond body_orientation;
        if (pc_start_imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0))) {
            body_orientation = Eigen::Quaterniond::Identity();
        } else {
            // Compute T_R^B = T_R^I * T_I^B where R = imu_reference_frame. If R = NWU our computation is done.
            const std::string imu_reference_frame = nh.param<std::string>("imu/reference_frame", "NED");
            body_orientation =
                    pc_start_imu.orientation * Eigen::Quaterniond(body_frames.frame_to_body("imu").rotation());
            if (imu_reference_frame == "NED") {
                // Since body orientation is relative NWU (map), compute T_NWU^B = T_NWU^NED * T_NED^B
                body_orientation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * body_orientation;
            }
        }
        const eigen_ros::Pose pose{
                Eigen::Vector3d(nh.param<double>("pose/position/x", 0.0), nh.param<double>("pose/position/y", 0.0),
                        nh.param<double>("pose/position/z", 0.0)),
                body_orientation, position_covariance, rotation_covariance};
        ROS_WARN_ONCE("MUST FIX: initial pose based on orientation but should be based on gravity. Can't use "
                      "orientation for 6-axis IMU");
        const Eigen::Matrix3d linear_twist_covariance =
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/linear_velocity", 1.0e-3), 2.0);
        const Eigen::Vector3d linear_velocity = Eigen::Vector3d{nh.param<double>("velocity/linear/x", 0.0),
                nh.param<double>("velocity/linear/y", 0.0), nh.param<double>("velocity/linear/z", 0.0)};
        ROS_WARN_ONCE("TODO FIX: angular velocity and cov must be converted from IMU frame to body frame");
        const eigen_ros::Twist twist{linear_velocity, pc_start_imu.angular_velocity, linear_twist_covariance,
                pc_start_imu.angular_velocity_covariance};
        world_odometry = eigen_ros::Odometry{pose, twist, pointcloud_start, "map", body_frames.body_frame_id()};

        // Set world state so first deskew is valid (TODO: clean up code duplication)
        world_state = gtsam::NavState(gtsam::Rot3(world_odometry.pose.orientation), world_odometry.pose.position,
                world_odometry.twist.linear);

        // Publish initial state
        auto odometry_msg = boost::make_shared<nav_msgs::Odometry>();
        eigen_ros::to_ros(*odometry_msg, world_odometry);
        initial_odometry_publisher.publish(odometry_msg);
        ROS_INFO_STREAM("Sent initial odometry message");
        previous_pointcloud_start = pointcloud_start;

        // Skew transform
        skew_transform = Eigen::Isometry3d::Identity();
        ROS_WARN_ONCE("DESIGN DECISION: Skew transform from initialisation procedure is missing. Using identity.");
    }

    // Compute scan end time
    pcl::PCLPointCloud2::Ptr deskewed_pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    const ros::Duration sweep_duration{pct::has_field(*msg, "t") ? pct::max<float>(*msg, "t") : 0};
    if (sweep_duration == ros::Duration(0)) {
        // No sweep required
        *deskewed_pointcloud = *msg;
        ROS_INFO_STREAM("Pointcloud does not require deskewing because sweep duration was 0.");
    } else {
        const ros::Time pointcloud_end = pointcloud_start + sweep_duration;
        ROS_INFO_STREAM("Pointcloud points span over " << sweep_duration << " s from " << pointcloud_start << " to "
                                                       << pointcloud_end);

        // Create new IMU integrator with new biases for deskewing
        gtsam::PreintegratedCombinedMeasurements preintegrated_imu_skew{preintegration_params, previous_imu_biases};

        // Sleep until IMU message received after scan end time
        ROS_INFO_STREAM("Waiting for IMU message past " << pointcloud_end);
        if (!protected_sleep(imu_mutex, 0.01, false, true,
                    [this, pointcloud_end]() { return last_preint_imu_timestamp < pointcloud_end; })) {
            return;
        };
        ROS_INFO_STREAM("Acquired end time IMU message at " << last_preint_imu_timestamp);

        // Compute state at pointcloud start time (body frame)
        gtsam::NavState pointcloud_start_state;
        if (initialised) {
            // Integrate IMU messages from previous pointcloud timestamp to current pointcloud timestamp
            integrate_imu(preintegrated_imu_skew, imu_buffer, previous_pointcloud_start, pointcloud_start);

            // Predict and reset integration
            pointcloud_start_state = preintegrated_imu_skew.predict(world_state, previous_imu_biases);
            preintegrated_imu_skew.resetIntegration();
        } else {
            pointcloud_start_state = world_state;
        }

        // Remove messages before pointcloud start timestamp
        delete_old_messages(pointcloud_start, imu_buffer);

        // Integrate IMU messages across pointcloud sweep
        integrate_imu(preintegrated_imu_skew, imu_buffer, pointcloud_start, pointcloud_end);
        imu_mutex.unlock();

        // Generate transform from preintegration, which is in the body frame.
        // T_{B_{i,s}}^{B_{i,e}} = T_{B_{i,s}}^{W} * T_{W}^{B_{i,e}}
        const gtsam::NavState pointcloud_end_state =
                preintegrated_imu_skew.predict(pointcloud_start_state, previous_imu_biases);
        skew_transform = eigen_gtsam::to_eigen<Eigen::Isometry3d>(pointcloud_start_state.pose()).inverse() *
                         eigen_gtsam::to_eigen<Eigen::Isometry3d>(pointcloud_end_state.pose());

        // Transform skew_transform from body frame to lidar frame
        skew_transform = eigen_ext::change_relative_transform_frame(skew_transform, body_frames.body_to_frame("lidar"));

        // Deskew configuration
        skew_transform = eigen_ext::to_transform(
                (deskew_translation ? skew_transform.translation() : Eigen::Vector3d{0.0, 0.0, 0.0}),
                (deskew_rotation ? skew_transform.rotation() : Eigen::Matrix3d::Identity()));

        // Deskew pointcloud
        const ros::WallTime tic = ros::WallTime::now();
        pct::deskew(skew_transform, sweep_duration.toSec(), *msg, *deskewed_pointcloud);
        ROS_INFO_STREAM("Deskewed pointcloud in " << (ros::WallTime::now() - tic).toSec() << " seconds.");
    }

    // Publish deskewed pointcloud
    deskewed_pointcloud_publisher.publish(deskewed_pointcloud);

    // Callback state for next pointcloud
    previous_pointcloud_start = pointcloud_start;
    initialised = true;
}

void Frontend::publish_stereo_data(const StereoData& data, const ros::Time& timestamp) {
    if (timestamp == ros::Time()) {
        left_image_publisher.publish(data.left_image);
        right_image_publisher.publish(data.right_image);
        left_info_publisher.publish(data.left_info);
        right_info_publisher.publish(data.right_info);
        ROS_INFO_STREAM("Re-publishing stereo data at its original timestamp");
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
        ROS_INFO_STREAM("Re-publishing stereo data with new timestamp = "
                        << timestamp << " (originally " << data.left_image->header.stamp
                        << ", diff = " << (data.left_image->header.stamp - timestamp).toSec() << " s)");
    }
}

void Frontend::stereo_callback(const sensor_msgs::ImageConstPtr& left_image,
        const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_info,
        const sensor_msgs::CameraInfoConstPtr& right_info) {
    std::lock_guard<std::mutex> guard{stereo_mutex};
    ROS_DEBUG_STREAM("Received stereo image at t = " << left_image->header.stamp);
    // Publish stereo data or add to queue
    if (publish_next_stereo) {
        publish_stereo_data(StereoData{left_image, right_image, left_info, right_info}, publish_next_stereo_timestamp);
        publish_next_stereo = false;
    } else {
        stereo_data.emplace_back(left_image, right_image, left_info, right_info);
    }
}

}
