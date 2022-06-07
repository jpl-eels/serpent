#include "serpent/registration.hpp"
#include "serpent/registration_methods.hpp"
#include "serpent/utilities.hpp"
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <pcl/common/transforms.h>

namespace serpent {

Registration::Registration():
    nh("~"), s2s_sync(10)
{
    // Publishers
    refined_transform_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("registration/transform", 1);

    // Subscribers
    pointcloud_subscriber.subscribe(nh, "normal_estimation/pointcloud", 10);
    transform_subscriber.subscribe(nh, "optimisation/imu_transform", 10);
    s2s_sync.connectInput(pointcloud_subscriber, transform_subscriber);
    s2s_sync.registerCallback(boost::bind(&Registration::s2s_callback, this, _1, _2));
    map_subscriber = nh.subscribe<pcl::PointCloud<pcl::PointNormal>>("mapping/local_map", 10,
            &Registration::s2m_callback, this);

    // Configuration
    nh.param<bool>("s2m/enabled", s2m_enabled, true);

    // Create registration methods
    s2s = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2s/");
    if (s2m_enabled) {
        s2m = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2m/");
    }

    // Debugging
    nh.param<bool>("debug/publish_registration_clouds", publish_registration_clouds, false);
    if (publish_registration_clouds) {
        debug_previous_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                "debug/previous_pointcloud", 1);
        debug_imu_guess_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                "debug/imu_guess_pointcloud", 1);
        debug_s2s_transformed_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                "debug/s2s_transformed_pointcloud", 1);
        debug_s2m_transformed_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                "debug/s2m_transformed_pointcloud", 1);
        nh.param<bool>("debug/publish_registration_clouds_alt", publish_registration_clouds_alt, false);
        if (publish_registration_clouds_alt) {
            debug_s2s_transformed_cloud_alt_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                    "debug/s2s_transformed_pointcloud_alt", 1);
            debug_s2m_transformed_cloud_alt_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>(
                    "debug/s2m_transformed_pointcloud_alt", 1);

        }
    }
}

void Registration::publish_refined_transform(const Eigen::Matrix4d transform,
        const Eigen::Matrix<double, 6, 6> covariance, const ros::Time& timestamp) {
    auto transform_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    transform_msg->header.stamp = timestamp;
    transform_msg->header.frame_id = "body_i-1"; // Child = "body_i"
    eigen_ros::to_ros(transform_msg->pose.pose, transform);
    eigen_ros::to_ros(transform_msg->pose.covariance, covariance);
    refined_transform_publisher.publish(transform_msg);
}

void Registration::s2s_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& current_pointcloud,
        const geometry_msgs::TransformStamped::ConstPtr& transform_msg) {    
    // Perform registration after first scan
    if (previous_pointcloud) {
        // Convert the transform message
        Eigen::Isometry3d transform = to_transform(eigen_ros::from_ros<eigen_ros::Pose>(transform_msg->transform));
        Eigen::Matrix4f tf_mat_float = transform.matrix().cast<float>();
        ROS_INFO_STREAM("S2S initial guess:\n" << tf_mat_float);

        if (publish_registration_clouds) {
            // Publish previous cloud (unfortunately a copy is required to correct the frame id)
            auto previous_pointcloud_frame_corrected = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(
                    *previous_pointcloud);
            previous_pointcloud_frame_corrected->header.frame_id = "body_i-1";
            debug_previous_cloud_publisher.publish(previous_pointcloud_frame_corrected);

            // Transform current scan to previous frame and publish
            auto imu_guess_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloud(*current_pointcloud, *imu_guess_pointcloud, tf_mat_float);
            imu_guess_pointcloud->header.frame_id = "body_i-1";
            debug_imu_guess_cloud_publisher.publish(imu_guess_pointcloud);
        }

        // Refine registration with scan to scan matching
        s2s->setInputSource(current_pointcloud);
        s2s->setInputTarget(previous_pointcloud);
        auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        ROS_INFO_STREAM("S2S aligning current cloud (" << current_pointcloud->size() << " points) to previous cloud ("
                << previous_pointcloud->size() << ")");
        const ros::WallTime tic = ros::WallTime::now();
        s2s->align(*registered_pointcloud, tf_mat_float);
        registered_pointcloud->header.frame_id = "body_i-1";
        const Eigen::Matrix4f s2s_transform_float = s2s->getFinalTransformation();
        const Eigen::Matrix4d s2s_transform = s2s_transform_float.cast<double>();
        ROS_INFO_STREAM("S2S finished in " << (ros::WallTime::now() - tic).toSec() << " s. "
                << registration_result(*s2s));
        if (!s2s->hasConverged()) {
            ROS_WARN("Scan to Scan did not converge");
        }

        // Optionally publish registered pointcloud
        if (publish_registration_clouds) {
            debug_s2s_transformed_cloud_publisher.publish(registered_pointcloud);

            if (publish_registration_clouds_alt) {
                // Alt: transform and publish
                auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
                pcl::transformPointCloud(*current_pointcloud, *registered_pointcloud_alt, s2s_transform_float);
                debug_s2s_transformed_cloud_alt_publisher.publish(registered_pointcloud_alt);
            }
        }

        if (s2m_enabled) {
            // Save results for S2M
            std::lock_guard<std::mutex> guard{s2s_mutex};
            s2m_pointclouds.emplace_back(current_pointcloud);
            s2s_registrations.emplace_back(s2s_transform);
        } else {
            // Publish the refined transform
            publish_refined_transform(s2s_transform, covariance_from_registration(*s2s),
                    pcl_conversions::fromPCL(current_pointcloud->header.stamp));
        }
    }

    previous_pointcloud = current_pointcloud;
}

void Registration::s2m_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& map) {
    // Wait for S2S transform and pointcloud
    if (!protected_sleep(s2s_mutex, 0.01, false, true, [this]()
            { return s2s_registrations.empty() || s2m_pointclouds.empty(); })) {
        return;
    };

    // Pop pointcloud and s2s registration from queues
    pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud = s2m_pointclouds.front();
    s2m_pointclouds.pop_front();
    Eigen::Isometry3d s2s_transform = s2s_registrations.front();
    s2s_registrations.pop_front();
    s2s_mutex.unlock();

    // Refine registration with scan to map matching
    Eigen::Matrix4f s2s_tf_mat_float = s2s_transform.matrix().cast<float>();
    s2m->setInputSource(pointcloud);
    s2m->setInputTarget(map);
    auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    ROS_INFO_STREAM("S2M Aligning current cloud (" << pointcloud->size() << " points) to map (" << map->size() << ")");
    const ros::WallTime tic = ros::WallTime::now();
    s2m->align(*registered_pointcloud, s2s_tf_mat_float);
    registered_pointcloud->header.frame_id = "body_i-1";
    const Eigen::Matrix4f s2m_transform_float = s2m->getFinalTransformation();
    const Eigen::Matrix4d s2m_transform = s2m_transform_float.cast<double>();
    ROS_INFO_STREAM("S2M finished in " << (ros::WallTime::now() - tic).toSec() << " s. " << registration_result(*s2m));
    if (!s2m->hasConverged()) {
        ROS_WARN("Scan to Map did not converge");
    }

    // Optionally publish registered pointcloud
    if (publish_registration_clouds) {
        debug_s2m_transformed_cloud_publisher.publish(registered_pointcloud);

        if (publish_registration_clouds_alt) {
            // Alt: transform and publish
            auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloud(*pointcloud, *registered_pointcloud_alt, s2m_transform_float);
            debug_s2m_transformed_cloud_alt_publisher.publish(registered_pointcloud_alt);
        }
    }

    // Publish the refined transform
    publish_refined_transform(s2m_transform, covariance_from_registration(*s2m),
            pcl_conversions::fromPCL(pointcloud->header.stamp));
}

}
