#ifndef CAMERA_TOOLS_SPLIT_IMAGE_HPP
#define CAMERA_TOOLS_SPLIT_IMAGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace camera_tools {

class PublishInfo {
public:
    explicit PublishInfo();

private:
    void callback(const sensor_msgs::ImageConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Subscriber image_subscriber;
    ros::Publisher camera_info_publisher;

    // Camera Info
    sensor_msgs::CameraInfo camera_info;
};

}

#endif
