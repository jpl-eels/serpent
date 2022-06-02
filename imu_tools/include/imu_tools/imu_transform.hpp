#ifndef IMU_TOOLS_IMU_TRANSFORM_HPP
#define IMU_TOOLS_IMU_TRANSFORM_HPP

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

class ImuTransform {
public:
    ImuTransform();

private:
    void transform(const sensor_msgs::ImuConstPtr& msg);

    // Configuration
    Eigen::Quaterniond rotation;
    bool overwrite_frame;
    std::string new_frame;

    // ROS objects
    ros::NodeHandle nh;
    ros::Publisher imu_publisher;
    ros::Subscriber imu_subscriber;
};

#endif
