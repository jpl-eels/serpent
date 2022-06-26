#ifndef NAVIGATION_TOOLS_INTEGRATE_IMU_HPP
#define NAVIGATION_TOOLS_INTEGRATE_IMU_HPP

#include <eigen_ros/eigen_ros.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <memory>

class IntegrateImu {
public:
    explicit IntegrateImu();

private:
    void integrate(const sensor_msgs::Imu::ConstPtr& msg);

    //// ROS Communication
    // Node handle
    ros::NodeHandle nh;
    // Odometry publisher
    ros::Publisher odometry_publisher;
    // Path publisher
    ros::Publisher path_publisher;
    // Transform subscriber
    ros::Subscriber imu_subscriber;

    //// Extrinsics
    // IMU to body frame extrinsic
    Eigen::Quaterniond imu_to_body_ext;
    // Body frame to IMU extrinsic (inverse of imu_to_body_ext)
    Eigen::Quaterniond body_to_imu_ext;

    //// IMU Integration
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;
    gtsam::imuBias::ConstantBias imu_bias;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> integrator;

    // State
    ros::Time integration_timestamp;
    gtsam::NavState initial_state;
    nav_msgs::Path path;
};

#endif
