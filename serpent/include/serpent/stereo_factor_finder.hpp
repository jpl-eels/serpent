#ifndef SERPENT_STEREO_FACTOR_FINDER_HPP
#define SERPENT_STEREO_FACTOR_FINDER_HPP

#include "serpent/StereoLandmarks.h"
#include "serpent/StereoTrackerStatistics.h"
#include "serpent/stereo_feature_tracker.hpp"
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
    
void to_ros(std::vector<serpent::StereoLandmark>& stereo_landmarks,
        const StereoFeatureTracker::LRKeyPointMatches& stereo_keypoint_matches);

void to_ros(serpent::StereoTrackerStatistics& msg, const StereoFeatureTracker::Statistics& statistics);

class StereoFactorFinder {
public:
    explicit StereoFactorFinder();

private:
    void stereo_callback(const sensor_msgs::ImageConstPtr& image_left, const sensor_msgs::ImageConstPtr& image_right,
            const sensor_msgs::CameraInfoConstPtr& info_left, const sensor_msgs::CameraInfoConstPtr& info_right);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher stereo_landmarks_publisher;
    ros::Publisher stereo_tracker_statistics_publisher;
    image_transport::ImageTransport it;
    message_filters::Subscriber<sensor_msgs::Image> left_image_subcriber;
    message_filters::Subscriber<sensor_msgs::Image> right_image_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_subcriber;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::CameraInfo> stereo_sync;
    //// Intermediate Results
    image_transport::Publisher extracted_keypoints_left_publisher;
    image_transport::Publisher extracted_keypoints_right_publisher;
    image_transport::Publisher sof_matches_left_publisher;
    image_transport::Publisher sof_matches_right_publisher;
    image_transport::Publisher stereo_filtered_matches_publisher;
    image_transport::Publisher new_matches_publisher;
    image_transport::Publisher tracked_matches_publisher;

    //// Debug
    bool print_stats;
    bool publish_stats;
    bool publish_intermediate_results;
    cv::DrawMatchesFlags keypoint_draw_flags;
    cv::DrawMatchesFlags match_draw_flags;

    // Tracker
    std::unique_ptr<StereoFeatureTracker> tracker;

    // Previous images (must be kept in scope)
    cv_bridge::CvImageConstPtr previous_left_image;
    cv_bridge::CvImageConstPtr previous_right_image;
};

}

#endif
