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

#include <eigen_ext/matrix.hpp>
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/eigen_ros.hpp>

#include "serpent/registration_degeneracy.hpp"
#include "serpent/utilities.hpp"

namespace serpent {

class Registration {
public:
    explicit Registration();

private:
    /**
     * @brief Compute the covariance matrix for a registration, given a base covariance. Also publishes the jacobian as
     * a debug message if configured to. The resultant covariance terms will not be less than the base covariance.
     *
     * @tparam PointSource
     * @tparam PointTarget
     * @param registration
     * @param base_covariance
     * @param jacobian_augmentation
     * @return Eigen::Matrix<double, 6, 6>
     */
    template<typename PointSource, typename PointTarget>
    Eigen::Matrix<double, 6, 6> covariance_from_registration(pcl::Registration<PointSource, PointTarget>& registration,
            const Eigen::Matrix<double, 6, 6>& base_covariance, const bool jacobian_augmentation);

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

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher refined_transform_publisher;
    ros::Publisher debug_previous_cloud_publisher;
    ros::Publisher debug_current_cloud_publisher;
    ros::Publisher debug_imu_guess_cloud_publisher;
    ros::Publisher debug_registration_jacobian_publisher;
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

    // Thread Management
    mutable std::mutex s2s_mutex;

    //// Configuration
    bool s2m_enabled;

    //// Debug Configuration
    bool publish_registration_clouds;
    bool publish_registration_clouds_alt;
    bool publish_registration_jacobian;

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
    // Scan-to-scan registration covariance base (rotation then translation order)
    Eigen::Matrix<double, 6, 6> s2s_covariance_base;
    // Scan-to-map registration covariance base (rotation then translation order)
    Eigen::Matrix<double, 6, 6> s2m_covariance_base;
    // Scan-to-Scan Jacobian Augmentation
    bool s2s_jacobian_augmentation;
    // Scan-to-Map Jacobian Augmentation
    bool s2m_jacobian_augmentation;
    // Body-to-lidar adjoint transform (rotation then translation order)
    Eigen::Matrix<double, 6, 6> body_lidar_transform_adjoint;
};

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> Registration::covariance_from_registration(
        pcl::Registration<PointSource, PointTarget>& registration, const Eigen::Matrix<double, 6, 6>& base_covariance,
        const bool jacobian_augmentation) {
    ROS_WARN_ONCE("DESIGN DECISION: Could use registration.getFitnessScore() in registration covariance?");
    Eigen::Matrix<double, 6, 6> covariance{base_covariance};
    if (jacobian_augmentation || publish_registration_jacobian) {
        ros::WallTime tic = ros::WallTime::now();
        int count;
        const Eigen::Matrix<float, 6, 1> jacobianf = point_to_plane_jacobian(registration, count);
        const Eigen::Matrix<double, 6, 1> jacobian = jacobianf.cast<double>();
        ROS_INFO_STREAM("Point to plane J:\n" << to_flat_string(jacobian));
        ROS_INFO_STREAM("Took " << (ros::WallTime::now() - tic).toSec() << " seconds to compute jacobian (" << count
                                << " correspondences).");
        if (publish_registration_jacobian) {
            debug_registration_jacobian_publisher.publish(eigen_ros::to_ros<std_msgs::Float64MultiArray>(jacobian));
        }
        if (jacobian_augmentation) {
            if (count > 0) {
                const Eigen::Matrix<double, 6, 6> jacobian_matrix_inv =
                        Eigen::DiagonalMatrix<double, 6>{jacobian}.inverse();
                covariance = jacobian_matrix_inv * base_covariance * jacobian_matrix_inv;
                covariance = covariance.cwiseMax(base_covariance);
            } else {
                ROS_WARN_STREAM("No correspondences found for computing jacobian.");
            }
        }
    }
    ROS_INFO_STREAM("Reg Covariance:\n" << covariance);
    return covariance;
}

template<typename PointSource, typename PointTarget>
std::string registration_result(pcl::Registration<PointSource, PointTarget>& registration) {
    std::stringstream ss;
    ss << "Converged: " << (registration.hasConverged() ? "True" : "False")
       << ", Fitness Score: " << registration.getFitnessScore() << ", Final Transform:\n"
       << registration.getFinalTransformation();
    return ss.str();
}

}

#endif
