#ifndef SERPENT_FRONTEND_HPP
#define SERPENT_FRONTEND_HPP

#include <gtsam/navigation/CombinedImuFactor.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <deque>
#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/imu.hpp>
#include <eigen_ros/odometry.hpp>
#include <memory>
#include <mutex>

#include "serpent/ImuBiases.h"

namespace serpent {

struct StereoData {
    StereoData(const sensor_msgs::ImageConstPtr left_image, const sensor_msgs::ImageConstPtr right_image,
            const sensor_msgs::CameraInfoConstPtr left_info, const sensor_msgs::CameraInfoConstPtr right_info)
        : left_image(left_image), right_image(right_image), left_info(left_info), right_info(right_info) {}

    sensor_msgs::ImageConstPtr left_image;
    sensor_msgs::ImageConstPtr right_image;
    sensor_msgs::CameraInfoConstPtr left_info;
    sensor_msgs::CameraInfoConstPtr right_info;

    inline ros::Time timestamp() const {
        return left_image->header.stamp;
    }
};

StereoData change_timestamp(const StereoData& data, const ros::Time& timestamp);

class Frontend {
public:
    explicit Frontend();

    std::string output_pointcloud_topic() const;

private:
    /**
     * @brief Barometer data callback.
     *
     * @param pressure
     */
    void barometer_callback(const sensor_msgs::FluidPressure::ConstPtr& pressure);

    /**
     * @brief Check if barometer data can be interpolated and published. Called when new barometer data arrives or a new
     * state timestamp is set.
     *
     * Assumes barometer data mutex is already acquired.
     *
     */
    void barometer_try_publish();

    /**
     * @brief Obtain the orientation from an IMU measurement in the body frame. Performs the required IMU reference
     * frame change (e.g. NED to NWU), and sensor to body transformation. Returns identity rotation if imu rotation is
     * invalid.
     *
     * @param imu
     * @return Eigen::Quaterniond
     */
    Eigen::Quaterniond body_frame_orientation(const eigen_ros::Imu& imu) const;

    /**
     * @brief IMU message callback
     *
     * Convert reference frame, add to buffer for deskewing, and publish the current odometry.
     *
     * @param msg
     */
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief Get an interpolated imu message at interp_timestamp. Requires that the IMU buffer contain an IMU
     * measurement both before and after the timestamp.
     *
     * @param interp_timestamp
     * @return Imu
     */
    eigen_ros::Imu interpolated_imu(const ros::Time interp_timestamp) const;

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
    ros::Publisher barometer_publisher;
    ros::Publisher deskewed_pointcloud_publisher;
    ros::Publisher imu_s2s_publisher;
    ros::Publisher initial_odometry_publisher;
    ros::Publisher odometry_publisher;
    ros::Publisher pose_publisher;
    ros::Subscriber barometer_subscriber;
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
            sensor_msgs::CameraInfo>
            stereo_sync;

    //// Thread Management
    // Protect imu_buffer
    mutable std::mutex imu_data_mutex;
    // Protect last_preint_imu_timestamp, preintegration_params, world_odometry, world_state, preintegrated_imu,
    //  imu_biases, imu_bias_timestamp
    mutable std::mutex preintegration_mutex;
    // Protect stereo_buffer
    mutable std::mutex stereo_data_mutex;
    // Protect barometer_buffer, barometer_timestamp_buffer
    mutable std::mutex barometer_data_mutex;

    // Body frames
    const eigen_ros::BodyFrames body_frames;
    // Map frame id
    std::string map_frame_id;

    //// Configuration
    // Overwrite IMU covariance flag
    bool overwrite_imu_covariance;
    // Overwrite IMU accelerometer covariance
    Eigen::Matrix3d accelerometer_covariance;
    // Overwrite IMU accelerometer covariance
    Eigen::Matrix3d gyroscope_covariance;
    // Overwrite barometer variance flag
    bool overwrite_barometer_variance;
    // Overwrite barometer variance
    double barometer_variance;
    // Deskew translation
    bool deskew_translation;
    // Deskew rotation
    bool deskew_rotation;
    // Stereo processing enabled
    bool stereo_enabled;
    // Only republish graph stereo frames
    bool only_graph_frames;
    // Barometer enabled
    bool barometer_enabled;

    //// IMU data
    // IMU FIFO buffer
    std::deque<eigen_ros::Imu> imu_buffer;

    //// PointCloud data
    // Initialisation has occurred (initial odometry sent and first pointcloud deskewed)
    bool initialised;
    // Previous pointcloud
    pcl::PCLPointCloud2::ConstPtr previous_pointcloud;
    // Previous state time (corresponds to world_odometry.timestamp but is always available)
    ros::Time previous_state_time;

    //// Preintegration data
    // Last timestamp for IMU preintegration (either last IMU or optimised odometry timestamp)
    ros::Time last_preint_imu_timestamp;
    // Preintegration parameters
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;
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

    //// Stereo data
    // Stereo FIFO buffer
    std::deque<StereoData> stereo_buffer;

    //// Barometer data
    // Barometer FIFO buffer
    std::deque<sensor_msgs::FluidPressure::ConstPtr> barometer_buffer;
    // Barometer timestamp buffer holds the state timestamps for interpolation
    std::deque<ros::Time> barometer_timestamp_buffer;
};

}

#endif
