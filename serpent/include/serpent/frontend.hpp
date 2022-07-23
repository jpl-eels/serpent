#ifndef SERPENT_FRONTEND_HPP
#define SERPENT_FRONTEND_HPP

#include "serpent/ImuBiases.h"
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/imu.hpp>
#include <eigen_ros/odometry.hpp>
#include <Eigen/Geometry>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <deque>
#include <memory>
#include <mutex>

namespace serpent {

struct StereoData {
    StereoData(const sensor_msgs::ImageConstPtr left_image, const sensor_msgs::ImageConstPtr right_image,
            const sensor_msgs::CameraInfoConstPtr left_info, const sensor_msgs::CameraInfoConstPtr right_info):
        left_image(left_image), right_image(right_image), left_info(left_info), right_info(right_info) {}

    sensor_msgs::ImageConstPtr left_image;
    sensor_msgs::ImageConstPtr right_image;
    sensor_msgs::CameraInfoConstPtr left_info;
    sensor_msgs::CameraInfoConstPtr right_info;

    ros::Time timestamp() const {
        return left_image->header.stamp;
    }
};

class Frontend {
public:
    explicit Frontend();

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
     * @brief Save optimised of previous scan and update integrator with new biases.
     * 
     * @param optimised_odometry 
     * @param imu_biases 
     */
    void optimised_odometry_callback(const serpent::ImuBiases::ConstPtr& imu_biases_msg,
            const nav_msgs::Odometry::ConstPtr& optimised_odometry_msg);

    /**
     * @brief PointCloud message callback
     * 
     * Deskew pointcloud and publish IMU measurements from previous to current scan.
     * 
     * @param msg 
     */
    void pointcloud_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    /**
     * @brief Publish stereo data, optionally with a new timestamp. Changing timestamp requires a deep copy.
     * 
     * @param data 
     * @param timestamp if not ros::Time(), overwrites the data timestamp
     */
    void publish_stereo_data(const StereoData& data, const ros::Time& timestamp = ros::Time());
    
    /**
     * @brief Stereo image messages callback
     * 
     * @param left_image 
     * @param right_image 
     * @param left_info 
     * @param right_info 
     */
    void stereo_callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image,
            const sensor_msgs::CameraInfoConstPtr& left_info, const sensor_msgs::CameraInfoConstPtr& right_info);

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
    //// Stereo data comms
    image_transport::ImageTransport it;
    image_transport::Publisher left_image_publisher;
    image_transport::Publisher right_image_publisher;
    ros::Publisher left_info_publisher;
    ros::Publisher right_info_publisher;
    message_filters::Subscriber<sensor_msgs::Image> left_image_subcriber;
    message_filters::Subscriber<sensor_msgs::Image> right_image_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_subcriber;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::CameraInfo> stereo_sync;

    //// Thread Management
    mutable std::mutex imu_mutex;
    mutable std::mutex optimised_odometry_mutex;
    mutable std::mutex stereo_mutex;

    // Body frames
    const eigen_ros::BodyFrames body_frames;

    //// Configuration
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

    //// Stereo
    // Stereo processing enabled
    bool stereo_enabled;
    // Flag for stereo thread to publish next data
    bool publish_next_stereo;
    // Timestamp for stereo thread to publish next data
    ros::Time publish_next_stereo_timestamp;
    // Stereo data
    std::deque<StereoData> stereo_data;
};

}

#endif
