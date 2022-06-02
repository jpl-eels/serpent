#ifndef IMU_TOOLS_IMU_ANALYSIS_HPP
#define IMU_TOOLS_IMU_ANALYSIS_HPP

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

class ImuAnalysis {
public:
    ImuAnalysis();

private:
    void analyse(const sensor_msgs::ImuConstPtr& msg);

    // Configuration
    bool apply_transform;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    std::string fixed_frame;
    std::string transformed_frame;

    // ROS objects
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Subscriber imu_subscriber;
};

#endif
