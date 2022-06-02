#include "serpent/li_frontend.hpp"
#include "serpent/utilities.hpp"
#include "serpent/ImuArray.h"
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/nav_msgs.hpp>
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
}

void LIFrontend::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Convert IMU to pointcloud frame using extrinsics
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    imu.change_frame(imu_to_body_ext, body_to_imu_ext);
    ROS_WARN_ONCE("TODO: IMU change of frame currently breaks covariance. Need to find out how to correctly change"
            " covariance reference frames");
    ROS_WARN_ONCE("TODO: Sensor covariance should be before change of frame, once covariance change of frame is fixed");
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
        gtsam::Matrix15 state_covariance = preintegrated_imu->preintMeasCov(); // rot, pos, vel, accel, gyro
        Eigen::Matrix<double, 6, 6> pose_covariance, twist_covariance;
        pose_covariance = reorder_pose_covariance(state_covariance.block<6, 6>(0, 0));
        // pose_covariance << state_covariance.block<3, 3>(3, 3), state_covariance.block<3, 3>(3, 0),
        //         state_covariance.block<3, 3>(0, 3), state_covariance.block<3, 3>(0, 0);
        twist_covariance << state_covariance.block<3, 3>(6, 6), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                imu.angular_velocity_covariance;
        */
        Eigen::Vector3d angular_velocity = imu.angular_velocity + imu_biases.gyroscope();
        
        // Publish current state as odometry output
        auto odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = imu.timestamp;
        odometry->header.frame_id = "map";
        odometry->child_frame_id = "body";
        eigen_ros::to_ros(odometry->pose.pose.position, state.position());
        eigen_ros::to_ros(odometry->pose.pose.orientation, state.attitude().toQuaternion());
        // TODO: add pose_covariance
        // eigen_ros::to_ros(odometry->pose.covariance, pose_covariance);
        eigen_ros::to_ros(odometry->twist.twist.linear, state.velocity());
        eigen_ros::to_ros(odometry->twist.twist.angular, angular_velocity);
        // TODO: add twist_covariance
        // eigen_ros::to_ros(odometry->twist.covariance, twist_covariance);
        ROS_WARN_ONCE("TODO: add correct covariances to IMU-rate odometry");
        odometry_publisher.publish(odometry);

        // Publish body_i-1 to body (IMU-rate frame) TF
        const gtsam::NavState incremental_state = preintegrated_imu->predict(gtsam::NavState(), imu_biases);
        geometry_msgs::TransformStamped incremental_tf;
        incremental_tf.header.stamp = imu.timestamp;
        incremental_tf.header.frame_id = "body_i-1";
        incremental_tf.child_frame_id = "body";
        eigen_ros::to_ros(incremental_tf.transform.translation, incremental_state.position());
        eigen_ros::to_ros(incremental_tf.transform.rotation, incremental_state.attitude().toQuaternion());
        tf_broadcaster.sendTransform(incremental_tf);
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
        Eigen::Matrix<double, 6, 6> pose_covariance;
        pose_covariance <<
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/position", 1.0e-3), 2.0),
                Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/rotation", 1.0e-3), 2.0);
        Eigen::Quaterniond imu_orientation = imu.orientation.isApprox(Eigen::Quaterniond(0, 0, 0, 0)) ?
                Eigen::Quaterniond::Identity() : imu.orientation;
        eigen_ros::Pose pose{Eigen::Vector3d(nh.param<double>("pose/position/x", 0.0),
                nh.param<double>("pose/position/y", 0.0), nh.param<double>("pose/position/z", 0.0)),
                imu_orientation, pose_covariance};
        ROS_WARN_ONCE("MUST FIX: initial pose based on orientation but should be based on gravity. Can't use"
                " orientation for 6-axis IMU");
        Eigen::Matrix<double, 6, 6> twist_covariance;
        twist_covariance <<
                Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("prior_noise/linear_velocity", 1.0e-3), 2.0),
                Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), imu.angular_velocity_covariance;
        const Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
        ROS_WARN_ONCE("DESIGN DECISION: Can we estimate initial linear velocity through initialisation procedure?");
        const eigen_ros::Twist twist{linear_velocity, imu.angular_velocity, twist_covariance};
        const eigen_ros::Odometry odometry{pose, twist, pointcloud_start, "map", "body_i"};

        // Publish initial state
        auto odometry_msg = boost::make_shared<nav_msgs::Odometry>();
        eigen_ros::to_ros(*odometry_msg, odometry);
        initial_odometry_publisher.publish(odometry_msg);
        ROS_INFO_STREAM("Sent initial odometry message");
        previous_pointcloud_start = pointcloud_start;

        // Deskew transform
        deskew_transform = Eigen::Quaterniond::Identity();
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
        deskew_transform = Eigen::Quaterniond::Identity();
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
            const gtsam::NavState sweep_state = preintegrated_imu_over_scan.predict(gtsam::NavState(), imu_biases);
            deskew_transform = Eigen::Translation<double, 3>(sweep_state.pose().translation()) *
                    sweep_state.pose().rotation().toQuaternion();
        }
    }

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

    // Publish map to body_i-1 TF
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = optimised_odometry_msg->header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "body_i-1";
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
