#ifndef SERPENT_LI_FRONTEND_HPP
#define SERPENT_LI_FRONTEND_HPP

#include "serpent/ImuBiases.h"
#include <eigen_ros/nav_msgs.hpp>
#include <eigen_ros/sensor_msgs.hpp>
#include <Eigen/Geometry>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <deque>
#include <memory>
#include <mutex>

namespace serpent {

class LIFrontend {
public:
    explicit LIFrontend();

private:
    /**
     * @brief IMU message callback
     * 
     * Convert reference frame, add to buffer for deskewing, and publish the current odometry.
     * 
     * @param msg 
     */
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief PointCloud message callback
     * 
     * Deskew pointcloud and publish IMU measurements from previous to current scan.
     * 
     * @param msg 
     */
    void pointcloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg);
    
    /**
     * @brief Save optimised of previous scan and update integrator with new biases.
     * 
     * @param optimised_odometry 
     * @param imu_biases 
     */
    void optimised_odometry_callback(const serpent::ImuBiases::ConstPtr& imu_biases_msg,
            const nav_msgs::Odometry::ConstPtr& optimised_odometry_msg);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher deskewed_pointcloud_publisher;
    ros::Publisher imu_s2s_publisher;
    ros::Publisher initial_odometry_publisher;
    ros::Publisher odometry_publisher;
    ros::Subscriber imu_subscriber;
    ros::Subscriber pointcloud_subscriber;
    message_filters::Subscriber<serpent::ImuBiases> imu_biases_subscriber;
    message_filters::Subscriber<nav_msgs::Odometry> optimised_odometry_subscriber;
    message_filters::TimeSynchronizer<serpent::ImuBiases, nav_msgs::Odometry> optimised_odometry_sync;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    //// Thread Management
    mutable std::mutex imu_mutex;
    mutable std::mutex optimised_odometry_mutex;

    //// Configuration
    // IMU to body frame extrinsic
    Eigen::Quaterniond imu_to_body_ext;
    // Body frame to IMU extrinsic (inverse of imu_to_body_ext)
    Eigen::Quaterniond body_to_imu_ext;
    // Overwrite IMU covariance flag
    bool overwrite_imu_covariance;
    // Overwrite IMU accelerometer covariance
    Eigen::Matrix3d overwrite_accelerometer_covariance;
    // Overwrite IMU accelerometer covariance
    Eigen::Matrix3d overwrite_gyroscope_covariance;

    //// IMU thread
    // IMU FIFO buffer
    std::deque<eigen_ros::Imu> imu_buffer;

    //// PointCloud thread
    // Initialisation has occurred
    bool initialised;
    // Previous pointcloud start time for pointcloud callback
    ros::Time previous_pointcloud_start;
    
    // Preintegration parameters
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;

    //// Optimised Odometry thread
    // Body frame with respect to world frame
    eigen_ros::Odometry world_odometry;
    // Body frame state for preintegration
    gtsam::NavState world_state;
    // IMU preintegration
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_imu;
    // IMU preintegration bias
    gtsam::imuBias::ConstantBias imu_biases;
    // IMU bias timestamp
    ros::Time imu_bias_timestamp;

    //// All threads
    // Last timestamp for IMU preintegration (either last IMU timestamp or lidar timestamp after reset)
    ros::Time last_preint_imu_timestamp;
};

}

#endif
