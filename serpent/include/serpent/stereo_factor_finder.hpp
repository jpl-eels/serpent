#ifndef SERPENT_STEREO_FACTOR_FINDER_HPP
#define SERPENT_STEREO_FACTOR_FINDER_HPP

#include "serpent/match_filters.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
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
    image_transport::Publisher image_matches_publisher;

    // Debug
    image_transport::Publisher prev_left_publisher;
    image_transport::Publisher sof_left_publisher;
    image_transport::Publisher prev_right_publisher;
    image_transport::Publisher sof_right_publisher;

    //// Visualisation
    cv::DrawMatchesFlags draw_feature_flag;

    //// Algorithms
    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::Feature2D> descriptor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Ptr<serpent::DistanceMatchFilter> distance_filter;
    cv::Ptr<serpent::StereoMatchFilter> stereo_filter;
    cv::Ptr<cv::SparseOpticalFlow> sparse_optical_flow;

    //// Previous frame data
    cv_bridge::CvImageConstPtr prev_image_left;
    cv_bridge::CvImageConstPtr prev_image_right;
    cv::Mat prev_points_left_mat;
    cv::Mat prev_points_right_mat;

    // Test: previous frames with keypoints
    cv_bridge::CvImage prev_output_image_left;
    cv_bridge::CvImage prev_output_image_right;
};

}

#endif
