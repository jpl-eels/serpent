#ifndef SERPENT_STEREO_FACTOR_FINDER_HPP
#define SERPENT_STEREO_FACTOR_FINDER_HPP

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace serpent {

class StereoFactorFinder {
public:
    explicit StereoFactorFinder();

private:
    void stereo_callback(const sensor_msgs::ImageConstPtr& image_left, const sensor_msgs::ImageConstPtr& image_right,
            const sensor_msgs::CameraInfoConstPtr& info_left, const sensor_msgs::CameraInfoConstPtr& info_right);

    //// ROS Communication
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    message_filters::Subscriber<sensor_msgs::Image> left_image_subcriber;
    message_filters::Subscriber<sensor_msgs::Image> right_image_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_subcriber;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::CameraInfo> stereo_sync;
    image_transport::Publisher left_image_publisher;
    ros::Publisher left_info_publisher;
    image_transport::Publisher right_image_publisher;
    ros::Publisher right_info_publisher;

    //// Feature detectors
    cv::Ptr<cv::Feature2D> detector;
    cv::DrawMatchesFlags draw_feature_flag;
};

}

#endif
