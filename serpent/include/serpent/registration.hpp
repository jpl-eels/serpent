#ifndef SERPENT_REGISTRATION_HPP
#define SERPENT_REGISTRATION_HPP

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Dense>
#include <eigen_ext/matrix.hpp>
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/eigen_ros.hpp>

#include "serpent/registration_covariance_estimator.hpp"
#include "serpent/utilities.hpp"

namespace serpent {

class Registration {
public:
    explicit Registration();

private:
    enum CovarianceEstimationMethod {
        CONSTANT,
        POINT_TO_POINT_LINEARISED,
        POINT_TO_POINT_NONLINEAR,
        POINT_TO_PLANE_LINEARISED,
        POINT_TO_PLANE_NONLINEAR
    };

    template<typename PointSource, typename PointTarget, typename Scalar>
    Eigen::Matrix<double, 6, 6> covariance_from_registration(
            pcl::Registration<PointSource, PointTarget, Scalar>& registration);

    /**
     * @brief Publish the registration-refined transform with covariance
     *
     * @param transform
     * @param covariance
     * @param timestamp
     */
    void publish_refined_transform(const Eigen::Matrix4d transform, const Eigen::Matrix<double, 6, 6> covariance,
            const ros::Time& timestamp);

    /**
     * @brief Scan-to-Scan Registration
     *
     * Register scan to the previous scan (with initial guess). Then request a map region at the refined pose.
     *
     * @param pointcloud_msg
     * @param transform_msg
     */
    void s2s_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pointcloud_msg,
            const geometry_msgs::TransformStamped::ConstPtr& transform_msg);

    /**
     * @brief Scan-to-Map Registration
     *
     * Register scan to a map region (with initial guess from Scan-to-Scan registration).
     *
     * @param msg
     */
    void s2m_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& msg);

    /**
     * @brief Convert string to covariance estimation method enum.
     *
     * @param string
     * @return CovarianceEstimationMethod
     */
    CovarianceEstimationMethod to_covariance_estimation_method(const std::string& string) const;

    std::string to_string(const CovarianceEstimationMethod method) const;

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher refined_transform_publisher;
    ros::Publisher debug_previous_cloud_publisher;
    ros::Publisher debug_current_cloud_publisher;
    ros::Publisher debug_imu_guess_cloud_publisher;
    ros::Publisher debug_s2s_transformed_cloud_publisher;
    ros::Publisher debug_s2m_transformed_cloud_publisher;
    ros::Publisher debug_s2s_transformed_cloud_alt_publisher;
    ros::Publisher debug_s2m_transformed_cloud_alt_publisher;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointNormal>> pointcloud_subscriber;
    message_filters::Subscriber<geometry_msgs::TransformStamped> transform_subscriber;
    message_filters::TimeSynchronizer<pcl::PointCloud<pcl::PointNormal>, geometry_msgs::TransformStamped> s2s_sync;
    ros::Subscriber map_subscriber;

    // Body frames
    const eigen_ros::BodyFrames body_frames;
    // Body-to-lidar adjoint transform (rotation then translation order)
    Eigen::Matrix<double, 6, 6> body_lidar_transform_adjoint;

    // Thread Management
    mutable std::mutex s2s_mutex;

    //// Configuration
    // Enable scan-to-map
    bool s2m_enabled;
    // Covariance estimation method
    CovarianceEstimationMethod covariance_estimation_method;
    // Constant point variance
    float point_variance;

    //// Debug Configuration
    bool publish_registration_clouds;
    bool publish_registration_clouds_alt;

    //// Pointclouds
    // Previous pointcloud for S2S registration
    pcl::PointCloud<pcl::PointNormal>::ConstPtr previous_pointcloud;
    // Pointcloud queue for S2M registration
    std::deque<pcl::PointCloud<pcl::PointNormal>::ConstPtr> s2m_pointclouds;

    // Scan-to-Scan Registration
    pcl::Registration<pcl::PointNormal, pcl::PointNormal>::Ptr s2s;
    // Scan-to-Map Registration
    pcl::Registration<pcl::PointNormal, pcl::PointNormal>::Ptr s2m;
    // Scan-to-Scan Registrations
    std::deque<Eigen::Isometry3d> s2s_registrations;

    //// Covariance estimation
    // Covariance estimator
    std::unique_ptr<RegistrationCovarianceEstimator<pcl::PointNormal, pcl::PointNormal>> covariance_estimator;
};

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> Registration::covariance_from_registration(
        pcl::Registration<PointSource, PointTarget, Scalar>& registration) {
    ROS_WARN_ONCE("DESIGN DECISION: Could use registration.getFitnessScore() in registration covariance?");
    if (!covariance_estimator) {
        throw std::runtime_error("covariance estimator not created");
    }

    // Covariance estimation
    ros::WallTime tic = ros::WallTime::now();
    Eigen::Matrix<double, 6, 6> covariance = covariance_estimator->estimate_covariance(registration, point_variance);
    ROS_INFO_STREAM("Took " << (ros::WallTime::now() - tic).toSec() << " seconds to compute covariance.");

    // Print additional information
    const CorrespondenceRegistrationCovarianceEstimator<pcl::PointNormal, pcl::PointNormal>* corr_cov_est =
            dynamic_cast<const CorrespondenceRegistrationCovarianceEstimator<pcl::PointNormal, pcl::PointNormal>*>(
                    covariance_estimator.get());
    if (corr_cov_est != nullptr) {
        ROS_INFO_STREAM(corr_cov_est->correspondence_count() << " correspondences were used in covariance estimation");
    }
    return covariance;
}

}

#endif
