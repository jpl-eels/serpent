#ifndef CAMERA_TOOLS_CHANGE_FRAME_ID_HPP
#define CAMERA_TOOLS_CHANGE_FRAME_ID_HPP

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace camera_tools {

class ChangeFrameId {
public:
    explicit ChangeFrameId();

private:
    void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);

    //// ROS Communications
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_subcriber;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> camera_sync;
    image_transport::ImageTransport it;
    image_transport::Publisher image_publisher;
    ros::Publisher camera_info_publisher;

    // Frame id
    std::string frame_id;
};

}

#endif
