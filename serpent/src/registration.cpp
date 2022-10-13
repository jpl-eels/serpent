#include "serpent/registration.hpp"

#include <pcl/common/transforms.h>

#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_ros/eigen_ros.hpp>

#include "serpent/registration_methods.hpp"
#include "serpent/utilities.hpp"

namespace serpent {

Registration::Registration()
    : nh("~"),
      s2s_sync(10) {
    // Publishers
    refined_transform_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("registration/transform", 1);

    // Subscribers
    pointcloud_subscriber.subscribe(nh, "normal_estimation/pointcloud", 10);
    transform_subscriber.subscribe(nh, "optimisation/imu_transform", 10);
    s2s_sync.connectInput(pointcloud_subscriber, transform_subscriber);
    s2s_sync.registerCallback(boost::bind(&Registration::s2s_callback, this, _1, _2));
    map_subscriber =
            nh.subscribe<pcl::PointCloud<pcl::PointNormal>>("mapping/local_map", 10, &Registration::s2m_callback, this);

    // Configuration
    nh.param<bool>("s2m/enabled", s2m_enabled, true);

    // Create registration methods
    s2s = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2s/");
    if (s2m_enabled) {
        s2m = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2m/");
    }

    // Create registration base covariances
    s2s_covariance_base << Eigen::Matrix3d::Identity() *
                                   std::pow(nh.param<double>("s2s/base_noise/rotation", 0.00174533), 2.0),
            Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("s2s/base_noise/translation", 1.0e-3), 2.0);
    s2m_covariance_base << Eigen::Matrix3d::Identity() *
                                   std::pow(nh.param<double>("s2m/base_noise/rotation", 0.00174533), 2.0),
            Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Identity() * std::pow(nh.param<double>("s2m/base_noise/translation", 1.0e-3), 2.0);
    nh.param<bool>("s2s/base_noise/jacobian_augmentation", s2s_jacobian_augmentation, false);
    nh.param<bool>("s2m/base_noise/jacobian_augmentation", s2m_jacobian_augmentation, false);

    // Compute body-lidar transform adjoint for covariance transformation
    body_lidar_transform_adjoint = eigen_ext::transform_adjoint(body_frames.body_to_frame("lidar"));

    // Debugging
    nh.param<bool>("debug/registration/publish_clouds", publish_registration_clouds, false);
    if (publish_registration_clouds) {
        debug_previous_cloud_publisher =
                nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/previous_pointcloud", 1);
        debug_current_cloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/current_pointcloud", 1);
        debug_imu_guess_cloud_publisher =
                nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/imu_guess_pointcloud", 1);
        debug_s2s_transformed_cloud_publisher =
                nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/s2s_transformed_pointcloud", 1);
        debug_s2m_transformed_cloud_publisher =
                nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/s2m_transformed_pointcloud", 1);
        nh.param<bool>("debug/registration/publish_clouds_alt", publish_registration_clouds_alt, false);
        if (publish_registration_clouds_alt) {
            debug_s2s_transformed_cloud_alt_publisher =
                    nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/s2s_transformed_pointcloud_alt", 1);
            debug_s2m_transformed_cloud_alt_publisher =
                    nh.advertise<pcl::PointCloud<pcl::PointNormal>>("debug/s2m_transformed_pointcloud_alt", 1);
        }
    }
}

void Registration::publish_refined_transform(const Eigen::Matrix4d transform,
        const Eigen::Matrix<double, 6, 6> covariance, const ros::Time& timestamp) {
    // Convert transform to body frame
    const Eigen::Isometry3d transform_lidar{transform};
    const Eigen::Isometry3d transform_body =
            eigen_ext::change_relative_transform_frame(transform_lidar, body_frames.body_to_frame("lidar"));
    const Eigen::Matrix<double, 6, 6> covariance_body = eigen_ext::change_covariance_frame(covariance,
            body_lidar_transform_adjoint);

    // Convert to ROS
    auto transform_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    transform_msg->header.stamp = timestamp;                       // Timestamp is at t_i
    transform_msg->header.frame_id = body_frames.body_frame_id();  // TF: body at t_i-1 -> body at t_i
    eigen_ros::to_ros(transform_msg->pose.pose, transform_body);
    eigen_ros::to_ros(transform_msg->pose.covariance, eigen_ext::reorder_covariance(covariance, 3));
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

        const std::string frame_id = body_frames.frame_id("lidar");
        if (publish_registration_clouds) {
            // Current pointcloud untransformed (must change timestamp for visualisation)
            auto current_pointcloud_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(*current_pointcloud);
            current_pointcloud_->header.stamp = previous_pointcloud->header.stamp;
            current_pointcloud_->header.frame_id = frame_id;
            debug_current_cloud_publisher.publish(current_pointcloud_);

            // Publish previous cloud (unfortunately a copy is required to correct the frame id)
            auto previous_pointcloud_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(*previous_pointcloud);
            previous_pointcloud_->header.frame_id = frame_id;
            debug_previous_cloud_publisher.publish(previous_pointcloud_);

            // Transform current scan to previous frame and publish (must change timestamp for visualisation)
            auto imu_guess_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloud(*current_pointcloud, *imu_guess_pointcloud, tf_mat_float);
            imu_guess_pointcloud->header.stamp = previous_pointcloud->header.stamp;
            imu_guess_pointcloud->header.frame_id = frame_id;
            debug_imu_guess_cloud_publisher.publish(imu_guess_pointcloud);
        }

        // Refine registration with scan to scan matching (must change timestamp for visualisation)
        s2s->setInputSource(current_pointcloud);
        s2s->setInputTarget(previous_pointcloud);
        auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        ROS_INFO_STREAM("S2S aligning current cloud (" << current_pointcloud->size() << " points) to previous cloud ("
                                                       << previous_pointcloud->size() << ")");
        const ros::WallTime tic = ros::WallTime::now();
        s2s->align(*registered_pointcloud, tf_mat_float);
        registered_pointcloud->header.stamp = previous_pointcloud->header.stamp;
        registered_pointcloud->header.frame_id = frame_id;
        const Eigen::Matrix4f s2s_transform_float = s2s->getFinalTransformation();
        const Eigen::Matrix4d s2s_transform = s2s_transform_float.cast<double>();
        ROS_INFO_STREAM(
                "S2S finished in " << (ros::WallTime::now() - tic).toSec() << " s. " << registration_result(*s2s));
        if (!s2s->hasConverged()) {
            ROS_WARN("Scan to Scan did not converge");
        }

        // Optionally publish registered pointcloud
        if (publish_registration_clouds) {
            debug_s2s_transformed_cloud_publisher.publish(registered_pointcloud);

            if (publish_registration_clouds_alt) {
                // Alt: transform and publish (must change timestamp for visualisation)
                auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
                pcl::transformPointCloud(*current_pointcloud, *registered_pointcloud_alt, s2s_transform_float);
                registered_pointcloud_alt->header.stamp = registered_pointcloud->header.stamp;
                registered_pointcloud_alt->header.frame_id = registered_pointcloud->header.frame_id;
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
            publish_refined_transform(s2s_transform,
                    covariance_from_registration(*s2s, s2s_covariance_base, s2s_jacobian_augmentation),
                    pcl_conversions::fromPCL(current_pointcloud->header.stamp));
        }
    }

    previous_pointcloud = current_pointcloud;
}

void Registration::s2m_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& map) {
    // Wait for S2S transform and pointcloud
    if (!protected_sleep(s2s_mutex, 0.01, false, true,
                [this]() { return s2s_registrations.empty() || s2m_pointclouds.empty(); })) {
        return;
    };

    // Pop pointcloud and s2s registration from queues
    pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud = s2m_pointclouds.front();
    s2m_pointclouds.pop_front();
    Eigen::Isometry3d s2s_transform = s2s_registrations.front();
    s2s_registrations.pop_front();
    s2s_mutex.unlock();

    // Refine registration with scan to map matching (must change timestamp for visualisation)
    Eigen::Matrix4f s2s_tf_mat_float = s2s_transform.matrix().cast<float>();
    s2m->setInputSource(pointcloud);
    s2m->setInputTarget(map);
    auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    ROS_INFO_STREAM("S2M Aligning current cloud (" << pointcloud->size() << " points) to map (" << map->size() << ")");
    const ros::WallTime tic = ros::WallTime::now();
    s2m->align(*registered_pointcloud, s2s_tf_mat_float);
    registered_pointcloud->header.stamp = map->header.stamp;
    registered_pointcloud->header.frame_id = body_frames.frame_id("lidar");
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
            // Alt: transform and publish (must change timestamp for visualisation)
            auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloud(*pointcloud, *registered_pointcloud_alt, s2m_transform_float);
            registered_pointcloud_alt->header.stamp = registered_pointcloud->header.stamp;
            registered_pointcloud_alt->header.frame_id = registered_pointcloud->header.frame_id;
            debug_s2m_transformed_cloud_alt_publisher.publish(registered_pointcloud_alt);
        }
    }

    // Publish the refined transform
    publish_refined_transform(s2m_transform,
            covariance_from_registration(*s2m, s2m_covariance_base, s2m_jacobian_augmentation),
            pcl_conversions::fromPCL(pointcloud->header.stamp));
}

}
