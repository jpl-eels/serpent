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

#include "serpent/registration_covariance.hpp"
#include "serpent/utilities.hpp"

namespace serpent {

template<typename PointT = pcl::PointNormal>
class Registration {
public:
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    using PCLRegistration = typename pcl::Registration<PointT, PointT, float>;
    using PCLRegistrationPtr = typename PCLRegistration::Ptr;

    explicit Registration();

private:
    using PointVarianceCovarianceFunction = typename Eigen::Matrix<double, 6, 6> (*)(PCLRegistration& registration,
            const double point_variance, int& correspondence_count);
    using RangeCovarianceFunction = typename Eigen::Matrix<double, 6, 6> (*)(PCLRegistration& registration,
            const double range_variance, int& correspondence_count);
    using RangeBiasCovarianceFunction = typename Eigen::Matrix<double, 6, 6> (*)(PCLRegistration& registration,
            const double range_variance, const double range_bias_variance, int& correspondence_count);

    Eigen::Matrix<double, 6, 6> covariance_from_registration(PCLRegistration& registration);

    template<typename Model>
    void create_covariance_function_bindings(const CovarianceEstimationMethod covariance_estimation_method);

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
    void s2s_callback(const PointCloudConstPtr& pointcloud_msg,
            const geometry_msgs::TransformStamped::ConstPtr& transform_msg);

    /**
     * @brief Scan-to-Map Registration
     *
     * Register scan to a map region (with initial guess from Scan-to-Scan registration).
     *
     * @param msg
     */
    void s2m_callback(const PointCloudConstPtr& msg);

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
    message_filters::Subscriber<PointCloud> pointcloud_subscriber;
    message_filters::Subscriber<geometry_msgs::TransformStamped> transform_subscriber;
    message_filters::TimeSynchronizer<PointCloud, geometry_msgs::TransformStamped> s2s_sync;
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
    // Covariance estimation model (for method = <CENSI, LLS>)
    CovarianceEstimationModel covariance_estimation_model;
    // Point covariance method
    PointCovarianceMethod point_covariance_method;
    // Constant point variance
    double point_variance;
    // Range variance
    double range_variance;
    // Range bias variance
    double range_bias_variance;

    //// Debug Configuration
    bool check_normals;
    bool publish_registration_clouds;
    bool publish_registration_clouds_alt;

    //// Pointclouds
    // Previous pointcloud for S2S registration
    PointCloudConstPtr previous_pointcloud;
    // Pointcloud queue for S2M registration
    std::deque<PointCloudConstPtr> s2m_pointclouds;

    // Scan-to-Scan Registration
    PCLRegistrationPtr s2s;
    // Scan-to-Map Registration
    PCLRegistrationPtr s2m;
    // Scan-to-Scan Registrations
    std::deque<Eigen::Isometry3d> s2s_registrations;

    //// Covariance estimation
    // Constant covariance (for method = <CONSTANT>)
    Eigen::Matrix<double, 6, 6> constant_covariance;
    // // Covariance estimation functions (for method = <CENSI, LLS>)
    PointVarianceCovarianceFunction point_variance_covariance;
    RangeCovarianceFunction range_covariance;
    RangeBiasCovarianceFunction range_bias_covariance;
};

}

#include "serpent/impl/registration.hpp"

#endif
