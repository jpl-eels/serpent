#include "serpent/li_frontend.hpp"
#include "serpent/utilities.hpp"
#include "serpent/ImuArray.h"
#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_gtsam/eigen_gtsam.hpp>
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <pointcloud_tools/pclpointcloud2_utilities.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>

namespace serpent {

LIFrontend::LIFrontend():
    nh("~"), optimised_odometry_sync(10), last_preint_imu_timestamp(0.0), initialised(false)
{
    // Publishers
    deskewed_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("frontend/deskewed_pointcloud", 1);
    imu_s2s_publisher = nh.advertise<serpent::ImuArray>("frontend/imu_s2s", 1);
    initial_odometry_publisher = nh.advertise<nav_msgs::Odometry>("frontend/initial_odometry", 1);
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("output/odometry", 1);

    // Subscribers
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>("input/imu", 1000, &LIFrontend::imu_callback, this);
    imu_biases_subscriber.subscribe(nh, "optimisation/imu_biases", 10);
    optimised_odometry_subscriber.subscribe(nh, "optimisation/odometry", 10);
    optimised_odometry_sync.connectInput(imu_biases_subscriber, optimised_odometry_subscriber);
    optimised_odometry_sync.registerCallback(boost::bind(&LIFrontend::optimised_odometry_callback, this, _1, _2));
    pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>("formatter/formatted_pointcloud", 100,
            &LIFrontend::pointcloud_callback, this);

    // Preintegration parameters
    nh.param<bool>("imu_noise/overwrite", overwrite_imu_covariance, false);
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
    if (overwrite_imu_covariance) {
        overwrite_accelerometer_covariance = Eigen::Matrix3d::Identity() *
                std::pow(nh.param<double>("imu_noise/accelerometer", 1.0e-3), 2.0);
        overwrite_gyroscope_covariance = Eigen::Matrix3d::Identity() *
                std::pow(nh.param<double>("imu_noise/gyroscope", 1.0e-3), 2.0);
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

void LIFrontend::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert IMU to pointcloud frame using extrinsics
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    if (overwrite_imu_covariance) {
        imu.linear_acceleration_covariance = overwrite_accelerometer_covariance;
        imu.angular_velocity_covariance = overwrite_gyroscope_covariance;
    }

    // Save transformed IMU message to buffer for deskewing
    std::lock_guard<std::mutex> guard(optimised_odometry_mutex);
    imu_mutex.lock();
    double dt = (imu.timestamp - last_preint_imu_timestamp).toSec(); // invalid but not used on 1st iteration
    last_preint_imu_timestamp = imu.timestamp;
    imu_buffer.push_back(imu);
    imu_mutex.unlock();

    if (preintegrated_imu) {
        // Integrate current IMU measurement into running integration to obtain current incremental transform
        preintegrated_imu->integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);

        // Calculate current state from previous state
        const gtsam::NavState state = preintegrated_imu->predict(world_state, imu_biases);
        /* TODO: Combine optimised odometry covariances (in world_odometry) with state_covariance from pre-integration
        const gtsam::Matrix15 state_covariance = preintegrated_imu->preintMeasCov(); // rot, pos, vel, accel, gyro
        const Eigen::Matrix<double, 6, 6> pose_covariance = state_covariance.block<6, 6>(0, 0);
        const Eigen::Matrix3d linear_velocity_covariance = state_covariance.block<3, 3>(6, 6);
        Eigen::Matrix<double, 6, 6> twist_covariance;
        twist_covariance << imu.angular_velocity_covariance, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                linear_velocity_covariance;
        */
        ROS_WARN_ONCE("TODO FIX: angular velocity must be converted from IMU frame to body frame - is this even"
                " possible? A rotation may be a good approximation.");
        const Eigen::Vector3d angular_velocity = body_frames.body_to_frame("imu").rotation()
                * (imu.angular_velocity + imu_biases.gyroscope());
        
        // Publish current state as odometry output
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = "map";
        odometry->child_frame_id = body_frames.body_frame();
        eigen_ros::to_ros(odometry->pose.pose.position, state.position());
        eigen_ros::to_ros(odometry->pose.pose.orientation, state.attitude().toQuaternion());
        // eigen_ros::to_ros(odometry->pose.covariance, eigen_ext::reorder_covariance(pose_covariance, 3));
        eigen_ros::to_ros(odometry->twist.twist.linear, state.velocity());
        eigen_ros::to_ros(odometry->twist.twist.angular, angular_velocity);
        // eigen_ros::to_ros(odometry->twist.covariance, eigen_ext::reorder_covariance(twist_covariance, 3));
        ROS_WARN_ONCE("TODO FIX: IMU-rate odometry is not valid");
        odometry_publisher.publish(odometry);

        // Publish map to body (IMU-rate frame) TF
        /*
        const Eigen::Isometry3d pose = eigen_gtsam::to_eigen<Eigen::Isometry3d>(state.pose());
        geometry_msgs::TransformStamped map_to_body_tf;
        map_to_body_tf.header.stamp = imu.timestamp;
        map_to_body_tf.header.frame_id = "map";
        map_to_body_tf.child_frame_id = body_frames.body_frame();
        eigen_ros::to_ros(map_to_body_tf.transform, pose);
        tf_broadcaster.sendTransform(map_to_body_tf);
        */
        ROS_WARN_ONCE("TODO: add IMU-rate TF back once confirmed it doesn't conflict with important TF publishing");
    }
}

void LIFrontend::pointcloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    // Save pointcloud start time
    const ros::Time pointcloud_start = pcl_conversions::fromPCL(msg->header.stamp);
    ROS_INFO_STREAM("Received pointcloud " << msg->header.seq << " with timestamp " << pointcloud_start);
    if (pct::empty(*msg)) {
        throw std::runtime_error("Handling of empty pointclouds not yet supported");
    }

    // Sleep until IMU message after scan start time
    ROS_INFO_STREAM("Waiting for start time IMU message past " << pointcloud_start);
    if (!protected_sleep(imu_mutex, 0.01, false, true, [this, pointcloud_start]()
            { return last_preint_imu_timestamp < pointcloud_start; })) {
        return;
    };
    ROS_INFO_STREAM("Acquired start time IMU message at " << last_preint_imu_timestamp);

    if (initialised) {
        // Accumulate and publish IMU measurements from previous to current scan, removing them from the buffer.
        auto imu_s2s = boost::make_shared<serpent::ImuArray>();
        imu_s2s->start_timestamp = previous_pointcloud_start;
        imu_s2s->end_timestamp = pointcloud_start;
        imu_s2s->measurements = old_messages_to_ros(pointcloud_start, imu_buffer);
        imu_s2s_publisher.publish(imu_s2s);
    }

    // Remove messages before pointcloud timestamp
    delete_old_messages(pointcloud_start, imu_buffer);
    eigen_ros::Imu imu = imu_buffer.front();
    imu_mutex.unlock();

    // Update preintegration parameters, with IMU noise at scan start time
    optimised_odometry_mutex.lock();
    ROS_WARN_ONCE("DESIGN DECISION: gravity from IMU measurements?");
    update_preintegration_params(*preintegration_params, imu.linear_acceleration_covariance,
            imu.angular_velocity_covariance);

    // If first time, perform initialisation procedure.
    Eigen::Isometry3d deskew_transform;
    if (!initialised) {
        // Compute initial state using user-defined position & yaw, and gravity-defined roll and pitch
        const Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Identity()
                * std::pow(nh.param<double>("prior_noise/position", 1.0e-3), 2.0);
        const Eigen::Matrix3d rotation_covariance = Eigen::Matrix3d::Identity()
                * std::pow(nh.param<double>("prior_noise/rotation", 1.0e-3), 2.0);
        const Eigen::Quaterniond body_orientation = imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0)) ?
                Eigen::Quaterniond::Identity() :
                Eigen::Quaterniond{(imu.orientation * body_frames.frame_to_body("imu")).rotation()};
        const eigen_ros::Pose pose{Eigen::Vector3d(nh.param<double>("pose/position/x", 0.0),
                nh.param<double>("pose/position/y", 0.0), nh.param<double>("pose/position/z", 0.0)),
                body_orientation, position_covariance, rotation_covariance};
        ROS_WARN_ONCE("MUST FIX: initial pose based on orientation but should be based on gravity. Can't use"
                " orientation for 6-axis IMU");
        const Eigen::Matrix3d linear_twist_covariance = Eigen::Matrix3d::Identity()
                * std::pow(nh.param<double>("prior_noise/linear_velocity", 1.0e-3), 2.0);
        const Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
        ROS_WARN_ONCE("TODO FIX: read linear velocity prior from mission file");
        ROS_WARN_ONCE("DESIGN DECISION: Can we estimate initial linear velocity through initialisation procedure?");
        ROS_WARN_ONCE("TODO FIX: angular velocity and cov must be converted from IMU frame to body frame");
        const eigen_ros::Twist twist{linear_velocity, imu.angular_velocity, linear_twist_covariance,
                imu.angular_velocity_covariance};
        const eigen_ros::Odometry odometry{pose, twist, pointcloud_start, "map", body_frames.body_frame()};

        // Publish initial state
        auto odometry_msg = boost::make_shared<nav_msgs::Odometry>();
        eigen_ros::to_ros(*odometry_msg, odometry);
        initial_odometry_publisher.publish(odometry_msg);
        ROS_INFO_STREAM("Sent initial odometry message");
        previous_pointcloud_start = pointcloud_start;

        // Deskew transform
        deskew_transform = Eigen::Isometry3d::Identity();
        ROS_WARN_ONCE("DESIGN DECISION: Deskew transform from initialisation procedure is missing. Using identity.");
    }

    // Wait until previous imu_biases received
    ROS_INFO_STREAM("Waiting for previous bias at " << previous_pointcloud_start);
    if (!protected_sleep(optimised_odometry_mutex, 0.01, true, false, [this]()
            { return imu_bias_timestamp != previous_pointcloud_start; })) {
        return;
    };
    ROS_INFO_STREAM("Acquired previous bias at " << imu_bias_timestamp);

    // Compute scan end time
    const ros::Duration sweep_duration = ros::Duration(pct::max_value<float>(*msg, "t"));
    if (sweep_duration == ros::Duration(0)) {
        // No sweep required
        deskew_transform = Eigen::Isometry3d::Identity();
        ROS_INFO_STREAM("Pointcloud does not require deskewing");
    } else {
        const ros::Time pointcloud_end = pointcloud_start + sweep_duration;
        ROS_INFO_STREAM("Pointcloud points span over " << sweep_duration << " from " << pointcloud_start << " to "
                << pointcloud_end);

        if (initialised) {
            // Create new IMU integrator with new biases for deskewing
            gtsam::PreintegratedCombinedMeasurements preintegrated_imu_over_scan{preintegration_params, imu_biases};

            // Sleep until IMU message received after scan end time
            ROS_INFO_STREAM("Waiting for IMU message past " << pointcloud_end);
            if (!protected_sleep(imu_mutex, 0.01, false, true, [this, pointcloud_end]()
                    { return last_preint_imu_timestamp < pointcloud_end; })) {
                return;
            };
            ROS_INFO_STREAM("Acquired end time IMU message at " << last_preint_imu_timestamp);

            // Integrate IMU messages across pointcloud sweep
            ros::Time last_timestamp = pointcloud_start;
            for (const auto& imu : imu_buffer) {
                if (imu.timestamp > pointcloud_start) {
                    const double dt = (std::min(imu.timestamp, pointcloud_end) - last_timestamp).toSec();
                    last_timestamp = imu.timestamp;
                    preintegrated_imu_over_scan.integrateMeasurement(imu.linear_acceleration, imu.angular_velocity, dt);
                    if (imu.timestamp > pointcloud_end) {
                        break;
                    }
                }
            }
            imu_mutex.unlock();

            // Generate transform from preintegration
            ROS_WARN_ONCE("TODO FIX: sweep state predicted for deskew is wrong, you cannot predict from"
                    " gtsam::NavState() because you lose velocity. Need to integrate from t_{s,i-1} to t_{s,i}, then"
                    " predict to get a state estimate (with velocity), then integrate from t_{s,i} to t_{e,i}.");
            const gtsam::NavState sweep_state = preintegrated_imu_over_scan.predict(gtsam::NavState(), imu_biases);
            deskew_transform = eigen_gtsam::to_eigen<Eigen::Isometry3d>(sweep_state.pose());
        }
    }

    // Transform deskew_transform from IMU frame to LIDAR frame
    deskew_transform = eigen_ext::change_relative_transform_frame(deskew_transform,
            body_frames.frame_to_frame("lidar", "imu"));

    // Deskew pointcloud
    pcl::PCLPointCloud2::Ptr deskewed_pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    try {
        pct::deskew(deskew_transform, sweep_duration.toSec(), *msg, *deskewed_pointcloud);
    } catch (const std::exception& ex) {
        ROS_WARN_ONCE("Pointcloud deskewing failed. Skipping deskewing. Error: %s", ex.what());
        *deskewed_pointcloud = *msg;
    } 

    // Publish deskewed pointcloud
    deskewed_pointcloud_publisher.publish(deskewed_pointcloud);

    // Callback state for next pointcloud
    previous_pointcloud_start = pointcloud_start;
    initialised = true;
}

void LIFrontend::optimised_odometry_callback(const serpent::ImuBiases::ConstPtr& imu_biases_msg,
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
    tf.child_frame_id = body_frames.body_frame();
    tf.transform.translation.x = optimised_odometry_msg->pose.pose.position.x;
    tf.transform.translation.y = optimised_odometry_msg->pose.pose.position.y;
    tf.transform.translation.z = optimised_odometry_msg->pose.pose.position.z;
    tf.transform.rotation = optimised_odometry_msg->pose.pose.orientation;
    tf_broadcaster.sendTransform(tf);

    // Save biases
    from_ros(*imu_biases_msg, imu_biases);
    imu_bias_timestamp = imu_biases_msg->header.stamp;

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

}
