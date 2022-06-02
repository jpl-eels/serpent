#ifndef SERPENT_REGISTRATION_HPP
#define SERPENT_REGISTRATION_HPP

#include <eigen_ros/nav_msgs.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace serpent {

class Registration {
public:
    explicit Registration();

private:
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
    ros::Publisher debug_imu_guess_cloud_publisher;
    ros::Publisher debug_s2s_transformed_cloud_publisher;
    ros::Publisher debug_s2m_transformed_cloud_publisher;
    ros::Publisher debug_s2s_transformed_cloud_alt_publisher;
    ros::Publisher debug_s2m_transformed_cloud_alt_publisher;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointNormal>> pointcloud_subscriber;
    message_filters::Subscriber<geometry_msgs::TransformStamped> transform_subscriber;
    message_filters::TimeSynchronizer<pcl::PointCloud<pcl::PointNormal>, geometry_msgs::TransformStamped> s2s_sync;
    ros::Subscriber map_subscriber;

    // Thread Management
    mutable std::mutex s2s_mutex;

    //// Configuration
    bool s2m_enabled;

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
};

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> covariance_from_registration(pcl::Registration<PointSource, PointTarget>& registration) {
    ROS_WARN_ONCE("DESIGN DECISION: Use fitness score in registration covariance?");
    return Eigen::Matrix<double, 6, 6>::Identity() * registration.getFitnessScore();
}

template<typename PointSource, typename PointTarget>
std::string registration_result(pcl::Registration<PointSource, PointTarget>& registration) {
    std::stringstream ss;
    ss << "Converged: " << (registration.hasConverged() ? "True" : "False") << ", Fitness Score: "
            << registration.getFitnessScore() << ", Final Transform:\n" << registration.getFinalTransformation();
    return ss.str();
}

}

#endif
