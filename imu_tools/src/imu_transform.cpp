#include "imu_tools/imu_transform.hpp"

#include <eigen_ros/eigen_ros.hpp>
#include <eigen_ros/sensor_msgs.hpp>

ImuTransform::ImuTransform()
    : nh("~") {
    imu_publisher =
            nh.advertise<sensor_msgs::Imu>(nh.param<std::string>("imu_transformed_topic", "imu_transformed"), 1000);
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>(nh.param<std::string>("imu_topic", "imu"), 1000,
            &ImuTransform::transform, this);
    rotation = Eigen::Quaterniond(eigen_ros::matrix3d_from_param(nh, "rotation"));
    nh.param<bool>("overwrite_frame", overwrite_frame, false);
    nh.param<std::string>("new_frame", new_frame, "");
}

void ImuTransform::transform(const sensor_msgs::ImuConstPtr& msg) {
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    imu.orientation = rotation * imu.orientation;
    imu.orientation_covariance = rotation * imu.orientation_covariance;
    ROS_WARN_ONCE("TODO FIX: Transformation of IMU covariance is incorrect.");
    if (overwrite_frame) {
        imu.frame = new_frame;
    }
    sensor_msgs::Imu transformed_msg = eigen_ros::to_ros<sensor_msgs::Imu>(imu);
    imu_publisher.publish(transformed_msg);
}
