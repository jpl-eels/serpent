#include "serpent/stereo_factor_finder.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/xfeatures2d/nonfree.hpp>

namespace serpent {

StereoFactorFinder::StereoFactorFinder():
    nh("~"), it(nh), stereo_sync(10)
{
    // Subscribers
    left_image_subcriber.subscribe(nh, "input/stereo/left/image", 10);
    right_image_subcriber.subscribe(nh, "input/stereo/right/image", 10);
    left_info_subcriber.subscribe(nh, "input/stereo/left/camera_info", 10);
    right_info_subcriber.subscribe(nh, "input/stereo/right/camera_info", 10);
    stereo_sync.connectInput(left_image_subcriber, right_image_subcriber,left_info_subcriber, right_info_subcriber);
    stereo_sync.registerCallback(boost::bind(&StereoFactorFinder::stereo_callback, this, _1, _2, _3, _4));

    // Publishers
    left_image_publisher = it.advertise("stereo/left/features/image", 1);
    left_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/left/features/camera_info", 1);
    right_image_publisher = it.advertise("stereo/right/features/image", 1);
    right_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/right/features/camera_info", 1);
}

void StereoFactorFinder::stereo_callback(const sensor_msgs::ImageConstPtr& image_left_msg,
        const sensor_msgs::ImageConstPtr& image_right_msg, const sensor_msgs::CameraInfoConstPtr& info_left,
        const sensor_msgs::CameraInfoConstPtr& info_right) {
    cv_bridge::CvImageConstPtr image_left = cv_bridge::toCvShare(image_left_msg);
    cv_bridge::CvImageConstPtr image_right = cv_bridge::toCvShare(image_right_msg);
    
    const int num_features{500};
    const float scale_factor{1.2};
    const int num_levels{8};
    const int edge_threshold{31};
    const int first_level{0};
    const int WTA_K{2};
    const cv::ORB::ScoreType score_type{cv::ORB::ScoreType::HARRIS_SCORE};
    const int patch_size{31};
    const int fast_threshold{20};
    cv::Ptr<cv::ORB> detector = cv::ORB::create(num_features, scale_factor, num_levels, edge_threshold, first_level,
            WTA_K, score_type, patch_size, fast_threshold);
    std::vector<cv::KeyPoint> key_points_left, key_points_right;

    ROS_INFO_STREAM("Starting detection");
    detector->detect(image_left->image, key_points_left);
    detector->detect(image_right->image, key_points_right);

    cv_bridge::CvImage output_image_left{image_left->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
    // cv_bridge::CvImage output_image_left{image_left->header, image_left->encoding, cv::Mat()};
    cv_bridge::CvImage output_image_right{image_right->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
    // cv_bridge::CvImage output_image_right{image_right->header, image_right->encoding, cv::Mat()};

    cv::drawKeypoints(image_left->image, key_points_left, output_image_left.image);
    cv::drawKeypoints(image_right->image, key_points_right, output_image_right.image);

    left_image_publisher.publish(output_image_left.toImageMsg());
    left_info_publisher.publish(info_left);
    right_image_publisher.publish(output_image_right.toImageMsg());
    right_info_publisher.publish(info_right);
    ROS_INFO_STREAM("Finished detection");

    // const int num_features{10};
    // const int num_octave_layers{3};
    // const double constrast_threshold{0.04};
    // const double edge_threshold{10.0};
    // const double sigma{1.6};
    // cv::Ptr<SIFT> detector = cv::xfeatures2d::SIFT::create(num_features, num_octave_layers, constrast_threshold,
    //         edge_threshold, sigma);
}

}
