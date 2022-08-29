#ifndef SERPENT_STEREO_FACTOR_FINDER_HPP
#define SERPENT_STEREO_FACTOR_FINDER_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Core>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "serpent/StereoFeatures.h"
#include "serpent/StereoTrackerStatistics.h"
#include "serpent/stereo_feature_tracker.hpp"

namespace serpent {

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> stereo_coordinate_to_point(const Eigen::Matrix<Scalar, 3, 1>& stereo_coordinate,
        const Eigen::Matrix<Scalar, 3, 3>& intrinsic, const Scalar baseline) {
    Eigen::Matrix<Scalar, 3, 1> point;
    const float u_diff = stereo_coordinate[0] - stereo_coordinate[1];
    if (u_diff == 0.f) {
        throw std::runtime_error("Stereo horizontal coordinates are identical, point is infinitely far.");
    }
    const float fx_b = intrinsic(0, 0) * baseline;
    point[0] = (stereo_coordinate[0] - intrinsic(0, 2)) * baseline / u_diff;
    point[1] = (stereo_coordinate[2] - intrinsic(1, 2)) * fx_b / (u_diff * intrinsic(1, 1));
    point[2] = fx_b / u_diff;
    return point;
}

pcl::PointXYZ stereo_coordinate_to_pcl_point(const float u_L, const float u_R, const float v,
        const Eigen::Matrix3f& intrinsic, const float baseline);

geometry_msgs::Point stereo_coordinate_to_ros_point(const float u_L, const float u_R, const float v,
        const Eigen::Matrix3f& intrinsic, const float baseline);

void to_ros(std::vector<serpent::StereoFeature>& stereo_features,
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
    ros::Publisher stereo_features_publisher;
    ros::Publisher stereo_tracker_statistics_publisher;
    ros::Publisher stereo_points_publisher;
    image_transport::ImageTransport it;
    message_filters::Subscriber<sensor_msgs::Image> left_image_subcriber;
    message_filters::Subscriber<sensor_msgs::Image> right_image_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_subcriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_subcriber;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,
            sensor_msgs::CameraInfo>
            stereo_sync;
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
    bool publish_points;
    cv::DrawMatchesFlags keypoint_draw_flags;
    cv::DrawMatchesFlags match_draw_flags;
    float baseline;

    // Tracker
    std::unique_ptr<StereoFeatureTracker> tracker;

    // Previous images (must be kept in scope)
    cv_bridge::CvImageConstPtr previous_left_image;
    cv_bridge::CvImageConstPtr previous_right_image;
};

}

#endif
