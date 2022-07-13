#include "serpent/stereo_factor_finder.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace serpent {

cv::ORB::ScoreType to_orb_score_type(const std::string& score_type) {
    if (score_type == "HARRIS_SCORE") {
        return cv::ORB::ScoreType::HARRIS_SCORE;
    } else if (score_type == "FAST_SCORE") {
        return cv::ORB::ScoreType::FAST_SCORE;
    }
    throw std::runtime_error("ScoreType \'" + score_type + "\' not recognised.");
}

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

    const std::string feature_type = nh.param<std::string>("stereo_factors/features/type", "ORB");
    if (feature_type == "ORB") {
        const int num_features = nh.param<int>("stereo_factors/features/orb/num_features", 500);
        const float scale_factor = nh.param<float>("stereo_factors/features/orb/scale_factor", 1.2);
        const int num_levels = nh.param<int>("stereo_factors/features/orb/num_levels", 8);
        const int edge_threshold = nh.param<int>("stereo_factors/features/orb/edge_threshold", 31);
        const int first_level = nh.param<int>("stereo_factors/features/orb/first_level", 0);
        const int wta_k = nh.param<int>("stereo_factors/features/orb/wta_k", 2);
        const cv::ORB::ScoreType score_type = to_orb_score_type(nh.param<std::string>(
                "stereo_factors/features/orb/score_type", "HARRIS_SCORE"));
        const int patch_size = nh.param<int>("stereo_factors/features/orb/patch_size", 31);
        const int fast_threshold = nh.param<int>("stereo_factors/features/orb/fast_threshold", 20);
        detector = cv::ORB::create(num_features, scale_factor, num_levels, edge_threshold, first_level, wta_k,
                score_type, patch_size, fast_threshold);
    } else if (feature_type == "SIFT") {
        #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
        const int num_features = nh.param<int>("stereo_factors/features/sift/num_features", 10);
        const int num_octave_layers = nh.param<int>("stereo_factors/features/sift/num_octave_layers", 3);
        const double constrast_threshold = nh.param<double>("stereo_factors/features/sift/constrast_threshold", 0.04);
        const double edge_threshold = nh.param<double>("stereo_factors/features/sift/edge_threshold", 10.0);
        const double sigma = nh.param<double>("stereo_factors/features/sift/sigma", 1.6);
        detector = cv::SIFT::create(num_features, num_octave_layers, constrast_threshold, edge_threshold, sigma);
        #else
        throw std::runtime_error("SIFT requires OpenCV >= 4.4.0. Current OpenCV version: "
                + std::to_string(CV_VERSION_MAJOR) + "." + std::to_string(CV_VERSION_MINOR) + "."
                + std::to_string(CV_VERSION_REVISION));
        #endif
    } else {
        throw std::runtime_error("Stereo factor feature type \'" + feature_type + "\' not recognised.");
    }
    if (nh.param<bool>("stereo_factors/features/visualisation/rich_keypoints", true)) {
        draw_feature_flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    } else {
        draw_feature_flag = cv::DrawMatchesFlags::DEFAULT;
    }
}

void StereoFactorFinder::stereo_callback(const sensor_msgs::ImageConstPtr& image_left_msg,
        const sensor_msgs::ImageConstPtr& image_right_msg, const sensor_msgs::CameraInfoConstPtr& info_left,
        const sensor_msgs::CameraInfoConstPtr& info_right) {
    cv_bridge::CvImageConstPtr image_left = cv_bridge::toCvShare(image_left_msg);
    cv_bridge::CvImageConstPtr image_right = cv_bridge::toCvShare(image_right_msg);

    ros::WallTime tic = ros::WallTime::now();
    std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    detector->detect(image_left->image, keypoints_left);
    ros::WallDuration detection_time = ros::WallTime::now() - tic;
    ROS_INFO_STREAM("Detection of " << keypoints_left.size() << " features in left camera took "
            << detection_time.toSec() << " s (" << detection_time.toSec() / keypoints_left.size()  << " s/feature).");
    tic = ros::WallTime::now();
    detector->detect(image_right->image, keypoints_right);
    detection_time = ros::WallTime::now() - tic;
    ROS_INFO_STREAM("Detection of " << keypoints_right.size() << " features in right camera took "
            << detection_time.toSec() << " s (" << detection_time.toSec() / keypoints_right.size()  << " s/feature).");
    cv_bridge::CvImage output_image_left{image_left->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
    cv_bridge::CvImage output_image_right{image_right->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
    cv::drawKeypoints(image_left->image, keypoints_left, output_image_left.image, cv::Scalar::all(-1),
            draw_feature_flag);
    cv::drawKeypoints(image_right->image, keypoints_right, output_image_right.image, cv::Scalar::all(-1),
            draw_feature_flag);

    left_image_publisher.publish(output_image_left.toImageMsg());
    left_info_publisher.publish(info_left);
    right_image_publisher.publish(output_image_right.toImageMsg());
    right_info_publisher.publish(info_right);
    ROS_INFO_STREAM("Finished detection");
}

}
