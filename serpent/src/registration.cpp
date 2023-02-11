#include "serpent/registration.hpp"

#include <pcl/common/transforms.h>

#include <eigen_ext/covariance.hpp>
#include <eigen_ext/geometry.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <pointcloud_tools/pclpointcloud_utilities.hpp>

#include "serpent/registration_methods.hpp"
#include "serpent/utilities.hpp"

namespace serpent {

Registration::Registration()
    : nh("serpent"),
      s2s_sync(10),
      body_frames("serpent") {
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
    const std::string scan_label = s2m_enabled ? "s2m" : "s2s";

    // Create registration methods
    s2s = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2s/");
    if (s2m_enabled) {
        s2m = registration_method<pcl::PointNormal, pcl::PointNormal>(nh, "s2m/");
    }

    // Covariance estimation configuration
    covariance_estimation_method =
            to_covariance_estimation_method(nh.param<std::string>(scan_label + "/covariance/method", "CENSI"));
    ROS_INFO_STREAM("Using registration covariance estimation method: " << to_string(covariance_estimation_method));
    switch (covariance_estimation_method) {
        case CONSTANT:
            constant_covariance = std::make_unique<ConstantCovariance>(
                    nh.param<double>(scan_label + "/covariance/constant/rotation", 0.0174533),
                    nh.param<double>(scan_label + "/covariance/constant/translation", 1.0e-2));
            break;
        case CENSI:  // Fallthrough
        case LLS:
            covariance_estimation_model = to_covariance_estimation_model(
                    nh.param<std::string>(scan_label + "/covariance/model", "POINT_TO_PLANE_LINEARISED"));
            // Create covariance estimator
            switch (covariance_estimation_model) {
                case POINT_TO_POINT_LINEARISED:
                    covariance_estimator =
                            std::make_unique<PointToPointIcpLinearised<pcl::PointNormal, pcl::PointNormal, float>>();
                    break;
                case POINT_TO_POINT_NONLINEAR:
                    covariance_estimator =
                            std::make_unique<PointToPointIcpNonlinear<pcl::PointNormal, pcl::PointNormal, float>>();
                    break;
                case POINT_TO_PLANE_LINEARISED:
                    covariance_estimator =
                            std::make_unique<PointToPlaneIcpLinearised<pcl::PointNormal, pcl::PointNormal, float>>();
                    break;
                case POINT_TO_PLANE_NONLINEAR:
                    covariance_estimator =
                            std::make_unique<PointToPlaneIcpNonlinear<pcl::PointNormal, pcl::PointNormal, float>>();
                    break;
                default:
                    throw std::runtime_error(
                            "CovarianceEstimationModel not handled. Cannot create covariance estimator");
            }
            ROS_INFO_STREAM(
                    "Using registration covariance estimation model: " << to_string(covariance_estimation_model));
            break;
        default:
            throw std::runtime_error("CovarianceEstimationMethod not handled. Cannot create covariance estimator");
    }

    // Point covariance method
    const std::string point_covariance_method =
            nh.param<std::string>(scan_label + "/covariance/point_covariance/method", "VOXEL_SIZE");
    float point_noise;
    if (point_covariance_method == "CONSTANT") {
        nh.param<float>(scan_label + "/covariance/point_covariance/constant", point_noise, 0.01f);
    } else if (point_covariance_method == "VOXEL_SIZE") {
        if (!nh.param<bool>("voxel_grid_filter/enabled", false)) {
            throw std::runtime_error("point_covariance/method \"" + point_covariance_method +
                                     "\" selected but voxel_grid_filter/enabled was false");
        }
        nh.param<float>("voxel_grid_filter/leaf_size", point_noise, 0.1f);
    } else {
        throw std::runtime_error("Unrecognised point_covariance/method \"" + point_covariance_method + "\"");
    }
    point_variance = std::pow(point_noise, 2.f);

    // Compute body-lidar transform adjoint for covariance transformation
    body_lidar_transform_adjoint = eigen_ext::transform_adjoint(body_frames.body_to_frame("lidar"));

    // Debugging
    nh.param<bool>("debug/registration/check_normals", check_normals, false);
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
    // Convert covariance to body frame, except in special case where covariance is infinite, in which case changing the
    // frame may result in NaN values.
    const Eigen::Matrix<double, 6, 6> covariance_body =
            covariance.allFinite() ? eigen_ext::change_covariance_frame(covariance, body_lidar_transform_adjoint)
                                   : covariance;

    // Convert to ROS
    auto transform_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    transform_msg->header.stamp = timestamp;                       // Timestamp is at t_i
    transform_msg->header.frame_id = body_frames.body_frame_id();  // TF: body at t_i-1 -> body at t_i
    eigen_ros::to_ros(transform_msg->pose.pose, transform_body);
    eigen_ros::to_ros(transform_msg->pose.covariance, eigen_ext::reorder_covariance(covariance_body, 3));
    refined_transform_publisher.publish(transform_msg);
}

void Registration::s2s_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& current_pointcloud,
        const geometry_msgs::TransformStamped::ConstPtr& transform_msg) {
    // Perform registration after first scan
    if (previous_pointcloud) {
        // Convert the incoming transform message, T_{L_i-1}^{L_i}
        const Eigen::Isometry3d transform =
                to_transform(eigen_ros::from_ros<eigen_ros::Pose>(transform_msg->transform));
        const Eigen::Matrix4f tf_mat_float = transform.matrix().cast<float>();
        ROS_DEBUG_STREAM("S2S initial guess:\n" << tf_mat_float);

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
            // By applying the T_{L_i-1}^{L_i} transform, the reference frame of the points change from {L_i} to {L_i-1}
            // as p_{L_i-1} = T_{L_i-1}^{L_i} * p_{L_i}.
            auto imu_guess_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloudWithNormals(*current_pointcloud, *imu_guess_pointcloud, tf_mat_float);
            imu_guess_pointcloud->header.stamp = previous_pointcloud->header.stamp;
            imu_guess_pointcloud->header.frame_id = frame_id;
            debug_imu_guess_cloud_publisher.publish(imu_guess_pointcloud);
        }

        // Refine registration with scan to scan matching (must change timestamp for visualisation)
        s2s->setInputSource(current_pointcloud);
        s2s->setInputTarget(previous_pointcloud);
        auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        const ros::WallTime tic = ros::WallTime::now();
        s2s->align(*registered_pointcloud, tf_mat_float);
        registered_pointcloud->header.stamp = previous_pointcloud->header.stamp;
        registered_pointcloud->header.frame_id = frame_id;
        const Eigen::Matrix4f s2s_transform_float = s2s->getFinalTransformation();  // T_{L_i-1}^{L_i}
        const Eigen::Matrix4d s2s_transform = s2s_transform_float.cast<double>();
        ROS_INFO_STREAM("S2S took " << (ros::WallTime::now() - tic).toSec() << " s to align current cloud ("
                                    << current_pointcloud->size() << " pts) to previous cloud ("
                                    << previous_pointcloud->size()
                                    << " pts). Converged: " << (s2s->hasConverged() ? "True" : "False")
                                    << ", Fitness Score: " << s2s->getFitnessScore() << ".");
        ROS_DEBUG_STREAM("S2S final transform:\n" << s2s_transform);
        ROS_WARN_COND(!s2s->hasConverged(), "Scan to Scan did not converge.");

        // Debug: Publish registered pointcloud
        if (publish_registration_clouds) {
            debug_s2s_transformed_cloud_publisher.publish(registered_pointcloud);

            if (publish_registration_clouds_alt) {
                // Alt: transform and publish (must change timestamp for visualisation)
                auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
                pcl::transformPointCloudWithNormals(*current_pointcloud, *registered_pointcloud_alt,
                        s2s_transform_float);
                registered_pointcloud_alt->header.stamp = registered_pointcloud->header.stamp;
                registered_pointcloud_alt->header.frame_id = registered_pointcloud->header.frame_id;
                debug_s2s_transformed_cloud_alt_publisher.publish(registered_pointcloud_alt);
            }
        }

        // Debug: Check normals of target cloud if using a point-to-plane method
        if (check_normals && (covariance_estimation_method == CovarianceEstimationMethod::CENSI ||
                    covariance_estimation_method == CovarianceEstimationMethod::LLS) &&
                (covariance_estimation_model == CovarianceEstimationModel::POINT_TO_PLANE_LINEARISED ||
                        covariance_estimation_model == CovarianceEstimationModel::POINT_TO_PLANE_NONLINEAR)) {
            const int unnormalised_normals_target = pct::check_normals(*previous_pointcloud);
            if (unnormalised_normals_target > 0) {
                throw std::runtime_error("Found " + std::to_string(unnormalised_normals_target) +
                                         " unnormalised normals in target cloud before covariance estimation.");
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
    if (!protected_sleep(s2s_mutex, 0.01, false, true,
                [this]() { return s2s_registrations.empty() || s2m_pointclouds.empty(); })) {
        return;
    };

    // Pop pointcloud and s2s registration from queues
    pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud = s2m_pointclouds.front();
    s2m_pointclouds.pop_front();
    Eigen::Isometry3d s2s_transform = s2s_registrations.front();  // T_{L_i-1}^{L_i}
    s2s_registrations.pop_front();
    s2s_mutex.unlock();

    // Refine registration with scan to map matching (must change timestamp for visualisation)
    Eigen::Matrix4f s2s_tf_mat_float = s2s_transform.matrix().cast<float>();
    s2m->setInputSource(pointcloud);
    s2m->setInputTarget(map);
    auto registered_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    const ros::WallTime tic = ros::WallTime::now();
    s2m->align(*registered_pointcloud, s2s_tf_mat_float);
    registered_pointcloud->header.stamp = map->header.stamp;
    registered_pointcloud->header.frame_id = body_frames.frame_id("lidar");
    const Eigen::Matrix4f s2m_transform_float = s2m->getFinalTransformation();  // T_{L_i-1}^{L_i}
    const Eigen::Matrix4d s2m_transform = s2m_transform_float.cast<double>();
    ROS_INFO_STREAM("S2M took " << (ros::WallTime::now() - tic).toSec() << " s to align current cloud ("
                                << pointcloud->size() << " pts) to map (" << map->size()
                                << " pts). Converged: " << (s2m->hasConverged() ? "True" : "False")
                                << ", Fitness Score: " << s2m->getFitnessScore() << ".");
    ROS_DEBUG_STREAM("S2M final transform:\n" << s2m_transform);
    ROS_WARN_COND(!s2m->hasConverged(), "Scan to Map did not converge.");

    // Optionally publish registered pointcloud
    if (publish_registration_clouds) {
        debug_s2m_transformed_cloud_publisher.publish(registered_pointcloud);

        if (publish_registration_clouds_alt) {
            // Alt: transform and publish (must change timestamp for visualisation)
            auto registered_pointcloud_alt = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
            pcl::transformPointCloudWithNormals(*pointcloud, *registered_pointcloud_alt, s2m_transform_float);
            registered_pointcloud_alt->header.stamp = registered_pointcloud->header.stamp;
            registered_pointcloud_alt->header.frame_id = registered_pointcloud->header.frame_id;
            debug_s2m_transformed_cloud_alt_publisher.publish(registered_pointcloud_alt);
        }
    }

    // Publish the refined transform
    publish_refined_transform(s2m_transform, covariance_from_registration(*s2m),
            pcl_conversions::fromPCL(pointcloud->header.stamp));
}

Registration::CovarianceEstimationMethod Registration::to_covariance_estimation_method(
        const std::string& string) const {
    if (string == "CONSTANT") {
        return CovarianceEstimationMethod::CONSTANT;
    } else if (string == "CENSI") {
        return CovarianceEstimationMethod::CENSI;
    } else if (string == "LLS") {
        return CovarianceEstimationMethod::LLS;
    }
    throw std::runtime_error("Could not convert from string \"" + string + "\" to CovarianceEstimationMethod");
}

std::string Registration::to_string(const CovarianceEstimationMethod method) const {
    switch (method) {
        case CovarianceEstimationMethod::CONSTANT:
            return "CONSTANT";
        case CovarianceEstimationMethod::CENSI:
            return "CENSI";
        case CovarianceEstimationMethod::LLS:
            return "LLS";
        default:
            throw std::runtime_error("CovarianceEstimationMethod could not be converted to string.");
    }
}

Registration::CovarianceEstimationModel Registration::to_covariance_estimation_model(const std::string& string) const {
    if (string == "POINT_TO_POINT_LINEARISED") {
        return CovarianceEstimationModel::POINT_TO_POINT_LINEARISED;
    } else if (string == "POINT_TO_POINT_NONLINEAR") {
        return CovarianceEstimationModel::POINT_TO_POINT_NONLINEAR;
    } else if (string == "POINT_TO_PLANE_LINEARISED") {
        return CovarianceEstimationModel::POINT_TO_PLANE_LINEARISED;
    } else if (string == "POINT_TO_PLANE_NONLINEAR") {
        return CovarianceEstimationModel::POINT_TO_PLANE_NONLINEAR;
    }
    throw std::runtime_error("Could not convert from string \"" + string + "\" to CovarianceEstimationModel");
}

std::string Registration::to_string(const CovarianceEstimationModel model) const {
    switch (model) {
        case CovarianceEstimationModel::POINT_TO_POINT_LINEARISED:
            return "POINT_TO_POINT_LINEARISED";
        case CovarianceEstimationModel::POINT_TO_POINT_NONLINEAR:
            return "POINT_TO_POINT_NONLINEAR";
        case CovarianceEstimationModel::POINT_TO_PLANE_LINEARISED:
            return "POINT_TO_PLANE_LINEARISED";
        case CovarianceEstimationModel::POINT_TO_PLANE_NONLINEAR:
            return "POINT_TO_PLANE_NONLINEAR";
        default:
            throw std::runtime_error("CovarianceEstimationModel could not be converted to string.");
    }
}

}
