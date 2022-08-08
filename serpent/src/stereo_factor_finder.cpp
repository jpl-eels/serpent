#include "serpent/stereo_factor_finder.hpp"
#include "serpent/StereoLandmarks.h"
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

cv::NormTypes to_norm_type(const std::string& norm_type) {
    if (norm_type == "NORM_INF") {
        return cv::NormTypes::NORM_INF;
    } else if (norm_type == "NORM_L1") {
        return cv::NormTypes::NORM_L1;
    } else if (norm_type == "NORM_L2") {
        return cv::NormTypes::NORM_L2;
    } else if (norm_type == "NORM_L2SQR") {
        return cv::NormTypes::NORM_L2SQR;
    } else if (norm_type == "NORM_HAMMING") {
        return cv::NormTypes::NORM_HAMMING;
    } else if (norm_type == "NORM_HAMMING2") {
        return cv::NormTypes::NORM_HAMMING2;
    } else if (norm_type == "NORM_TYPE_MASK") {
        return cv::NormTypes::NORM_TYPE_MASK;
    } else if (norm_type == "NORM_RELATIVE") {
        return cv::NormTypes::NORM_RELATIVE;
    } else if (norm_type == "NORM_MINMAX") {
        return cv::NormTypes::NORM_MINMAX;
    }
    throw std::runtime_error("NormType \'" + norm_type + "\' not recognised.");
}

int to_term_criteria_type(const std::string& term_criteria_type) {
    if (term_criteria_type == "COUNT" || term_criteria_type == "MAX_ITER") {
        return cv::TermCriteria::COUNT;
    } else if (term_criteria_type == "EPS") {
        return cv::TermCriteria::EPS;
    } else if (term_criteria_type == "COUNT+EPS" || term_criteria_type == "MAX_ITER+EPS") {
        return cv::TermCriteria::COUNT + cv::TermCriteria::EPS;
    }
    throw std::runtime_error("TermCriteria::Type \'" + term_criteria_type + "\' not recognised.");
}

cv::FastFeatureDetector::DetectorType to_fast_type(const std::string& fast_type) {
    if (fast_type == "TYPE_5_8") {
        return cv::FastFeatureDetector::DetectorType::TYPE_5_8;
    } else if (fast_type == "TYPE_7_12") {
        return cv::FastFeatureDetector::DetectorType::TYPE_7_12;
    } else if (fast_type == "TYPE_9_16") {
        return cv::FastFeatureDetector::DetectorType::TYPE_9_16;
    }
    throw std::runtime_error("FastFeatureDetector Type \'" + fast_type + "\' not recognised.");
}

template<typename Feature2DType>
cv::Ptr<Feature2DType> create_from_params(const ros::NodeHandle& nh);

template<>
cv::Ptr<cv::FastFeatureDetector> create_from_params<cv::FastFeatureDetector>(const ros::NodeHandle& nh) {
    const int threshold = nh.param<int>("stereo_factors/detector/fast/threshold", 10);
    const bool nonmax_suppression = nh.param<bool>("stereo_factors/detector/fast/nonmax_suppression", true);
    const cv::FastFeatureDetector::DetectorType type =
            to_fast_type(nh.param<std::string>("stereo_factors/detector/fast/type", "TYPE_9_16"));
    return cv::FastFeatureDetector::create(threshold, nonmax_suppression, type);
}

template<>
cv::Ptr<cv::GFTTDetector> create_from_params<cv::GFTTDetector>(const ros::NodeHandle& nh) {
    const int max_corners = nh.param<int>("stereo_factors/detector/gftt/max_corners", 1000);
    const double quality_level = nh.param<double>("stereo_factors/detector/gftt/quality_level", 0.01);
    const double min_distance = nh.param<double>("stereo_factors/detector/gftt/min_distance", 1.0);
    const int block_size = nh.param<int>("stereo_factors/detector/gftt/block_size", 3);
    const bool use_harris_detector = nh.param<bool>("stereo_factors/detector/gftt/use_harris_detector", false);
    const double k = nh.param<double>("stereo_factors/detector/gftt/k", 0.04);
    return cv::GFTTDetector::create(max_corners, quality_level, min_distance, block_size, use_harris_detector, k);
}

template<>
cv::Ptr<cv::ORB> create_from_params<cv::ORB>(const ros::NodeHandle& nh) {
    const int num_features = nh.param<int>("stereo_factors/detector/orb/num_features", 500);
    const float scale_factor = nh.param<float>("stereo_factors/detector/orb/scale_factor", 1.2);
    const int num_levels = nh.param<int>("stereo_factors/detector/orb/num_levels", 8);
    const int edge_threshold = nh.param<int>("stereo_factors/detector/orb/edge_threshold", 31);
    const int first_level = nh.param<int>("stereo_factors/detector/orb/first_level", 0);
    const int wta_k = nh.param<int>("stereo_factors/detector/orb/wta_k", 2);
    const cv::ORB::ScoreType score_type = to_orb_score_type(nh.param<std::string>(
            "stereo_factors/detector/orb/score_type", "HARRIS_SCORE"));
    const int patch_size = nh.param<int>("stereo_factors/detector/orb/patch_size", 31);
    const int fast_threshold = nh.param<int>("stereo_factors/detector/orb/fast_threshold", 20);
    return cv::ORB::create(num_features, scale_factor, num_levels, edge_threshold, first_level, wta_k, score_type,
            patch_size, fast_threshold);
}

#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
template<>
cv::Ptr<cv::SIFT> create_from_params<cv::SIFT>(const ros::NodeHandle& nh) {
    const int num_features = nh.param<int>("stereo_factors/detector/sift/num_features", 10);
    const int num_octave_layers = nh.param<int>("stereo_factors/detector/sift/num_octave_layers", 3);
    const double constrast_threshold = nh.param<double>("stereo_factors/detector/sift/constrast_threshold", 0.04);
    const double edge_threshold = nh.param<double>("stereo_factors/detector/sift/edge_threshold", 10.0);
    const double sigma = nh.param<double>("stereo_factors/detector/sift/sigma", 1.6);
    return cv::SIFT::create(num_features, num_octave_layers, constrast_threshold, edge_threshold, sigma);
}
#endif

void print_keypoint(const cv::KeyPoint& kp, const std::string& id = std::string()) {
    ROS_INFO_STREAM("Keypoint" << (id.empty() ? std::string() : " (" + id + ")") << ":"
            << "\n\tpt: " << kp.pt.x << ", " << kp.pt.y
            << "\n\tsize: " << kp.size
            << "\n\tangle: "<< kp.angle
            << "\n\tresponse: "<< kp.response
            << "\n\toctave: "<< kp.octave
            << "\n\tclass_id: "<< kp.class_id);
}

void to_ros(std::vector<serpent::StereoLandmark>& stereo_landmarks,
        const StereoFeatureTracker::LRKeyPointMatches& stereo_keypoint_matches) {
    stereo_landmarks.clear();
    for (std::size_t i = 0; i < stereo_keypoint_matches.matches.size(); ++i) {
        const auto& match = stereo_keypoint_matches.matches[i];
        serpent::StereoLandmark stereo_landmark;
        stereo_landmark.left_x = stereo_keypoint_matches.keypoints[0][match.queryIdx].pt.x;
        stereo_landmark.left_y = stereo_keypoint_matches.keypoints[0][match.queryIdx].pt.y;
        stereo_landmark.right_x = stereo_keypoint_matches.keypoints[1][match.trainIdx].pt.x;
        stereo_landmark.right_y = stereo_keypoint_matches.keypoints[1][match.trainIdx].pt.y;
        stereo_landmark.id = stereo_keypoint_matches.match_ids[i];
        stereo_landmarks.push_back(stereo_landmark);
    }
}

StereoFactorFinder::StereoFactorFinder():
    nh("~"), it(nh), stereo_sync(10)
{
    // Publishers
    stereo_landmarks_publisher = nh.advertise<serpent::StereoLandmarks>("stereo/landmarks", 1);

    // Subscribers
    left_image_subcriber.subscribe(nh, "stereo/left/image", 10);
    right_image_subcriber.subscribe(nh, "stereo/right/image", 10);
    left_info_subcriber.subscribe(nh, "stereo/left/camera_info", 10);
    right_info_subcriber.subscribe(nh, "stereo/right/camera_info", 10);
    stereo_sync.connectInput(left_image_subcriber, right_image_subcriber, left_info_subcriber, right_info_subcriber);
    stereo_sync.registerCallback(boost::bind(&StereoFactorFinder::stereo_callback, this, _1, _2, _3, _4));

    // Additional Publishers
    nh.param<bool>("stereo_factors/visualisation/publish_intermediate_results", publish_intermediate_results, false);
    if (publish_intermediate_results) {
        extracted_keypoints_left_publisher = it.advertise("stereo/left/extracted_keypoints/image", 1);
        extracted_keypoints_right_publisher = it.advertise("stereo/right/extracted_keypoints/image", 1);
        sof_matches_left_publisher = it.advertise("stereo/left/sof_matches/image", 1);
        sof_matches_right_publisher = it.advertise("stereo/right/sof_matches/image", 1);
        stereo_filtered_matches_publisher = it.advertise("stereo/stereo_filtered_matches/image", 1);
        new_matches_publisher = it.advertise("stereo/new_matches/image", 1);
        tracked_matches_publisher = it.advertise("stereo/tracked_matches/image", 1);
    }

    // Components of tracker
    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::SparseOpticalFlow> sparse_optical_flow;
    cv::Ptr<StereoMatchFilter> stereo_filter;
    cv::Ptr<StereoKeyPointMatcher> stereo_matcher;

    // Detector
    const std::string feature_type = nh.param<std::string>("stereo_factors/detector/type", "ORB");
    if (feature_type == "FAST") {
        detector = create_from_params<cv::FastFeatureDetector>(nh);
    } else if (feature_type == "GFTT") {
        detector = create_from_params<cv::GFTTDetector>(nh);
    } else if (feature_type == "ORB") {
        detector = create_from_params<cv::ORB>(nh);
    } else if (feature_type == "SIFT") {
        #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
        detector = create_from_params<cv::SIFT>(nh);
        #else
        throw std::runtime_error("SIFT requires OpenCV >= 4.4.0. Current OpenCV version: "
                + std::to_string(CV_VERSION_MAJOR) + "." + std::to_string(CV_VERSION_MINOR) + "."
                + std::to_string(CV_VERSION_REVISION));
        #endif
    } else {
        throw std::runtime_error("Stereo factor feature type \'" + feature_type + "\' not recognised.");
    }

    // Stereo Filter
    const float vertical_pixel_threshold = nh.param<float>(
            "stereo_factors/stereo_match_filter/vertical_pixel_threshold", 1.0);
    stereo_filter = serpent::StereoMatchFilter::create(vertical_pixel_threshold);

    // Sparse Optical Flow
    const std::string sparse_optical_flow_type = nh.param<std::string>("stereo_factors/sparse_optical_flow/type",
            "PyrLK");
    if (sparse_optical_flow_type == "PyrLK") {
        const std::vector<int> win_size_vec = nh.param<std::vector<int>>("stereo_factors/sparse_optical_flow/win_size",
                {{21, 21}});
        const cv::Size win_size = cv::Size{win_size_vec.at(0), win_size_vec.at(1)};
        const int max_level = nh.param<int>("stereo_factors/sparse_optical_flow/max_level", 3);
        const cv::TermCriteria term_criteria = cv::TermCriteria{to_term_criteria_type(
                nh.param<std::string>("stereo_factors/sparse_optical_flow/term_criteria/type", "COUNT+EPS")),
                nh.param<int>("stereo_factors/sparse_optical_flow/term_criteria/max_count", 30),
                nh.param<double>("stereo_factors/sparse_optical_flow/term_criteria/epsilon", 0.01)};
        const int flags = nh.param<int>("stereo_factors/sparse_optical_flow/flags", 0);
        const double min_eig_threshold = nh.param<double>("stereo_factors/sparse_optical_flow/min_eig_threshold", 0);
        sparse_optical_flow = cv::SparsePyrLKOpticalFlow::create(win_size, max_level, term_criteria, flags,
                min_eig_threshold);
    } else if (sparse_optical_flow_type == "RLOF") {
        throw std::runtime_error("Stereo factor sparse optical flow type \'" + sparse_optical_flow_type
                + "\' not yet implemented.");
    } else {
        throw std::runtime_error("Stereo factor sparse optical flow type \'" + sparse_optical_flow_type
                + "\' not recognised.");
    }

    // Stereo KeyPoint Matcher
    const cv::Size stereo_matcher_window{nh.param<int>("stereo_factors/stereo_keypoint_matcher/window_size/width", 3),
            nh.param<int>("stereo_factors/stereo_keypoint_matcher/window_size/height", 3)};
    const std::string stereo_matcher_cost_function_str =
            nh.param<std::string>("stereo_factors/stereo_keypoint_matcher/cost_function", "SAD");
    StereoKeyPointMatcher::MatchingCostFunction stereo_matcher_cost_function;
    if (stereo_matcher_cost_function_str == "SAD") {
        stereo_matcher_cost_function = &sum_of_absolute_differences;
    } else {
        throw std::runtime_error(stereo_matcher_cost_function_str + " not yet implemented");
    }
    stereo_matcher = StereoKeyPointMatcher::create(stereo_matcher_cost_function, stereo_matcher_window,
            nh.param<double>("stereo_factors/stereo_keypoint_matcher/vertical_pixel_threshold", 1.0));
    const double stereo_match_cost_threshold = nh.param<double>("stereo_factors/stereo_keypoint_matcher/cost_threshold",
            1.0);

    // Region of Interest
    cv::Rect2i roi;
    if (nh.param<bool>("stereo_mask/enabled", false)) {
        roi.height = nh.param<int>("stereo_mask/height", 0);
        roi.width = nh.param<int>("stereo_mask/width", 0);
        roi.x = nh.param<int>("stereo_mask/top_left_x", 0);
        roi.y = nh.param<int>("stereo_mask/top_left_y", 0);
    }

    // Create tracker
    const float new_feature_dist_threshold = nh.param<float>("stereo_factors/new_feature_dist_threshold", 5.0);
    tracker = std::make_unique<StereoFeatureTracker>(detector, sparse_optical_flow, stereo_filter, stereo_matcher,
            new_feature_dist_threshold, stereo_match_cost_threshold, roi);

    // Visualisation
    nh.param<bool>("stereo_factors/visualisation/print_stats", print_stats, false);
    if (nh.param<bool>("stereo_factors/visualisation/rich_keypoints", true)) {
        keypoint_draw_flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    } else {
        keypoint_draw_flags = cv::DrawMatchesFlags::DEFAULT;
    }
    if (nh.param<bool>("stereo_factors/visualisation/rich_matches", true)) {
        match_draw_flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    } else {
        match_draw_flags = cv::DrawMatchesFlags::DEFAULT;
    }
}

void publish_image(image_transport::Publisher& publisher, const cv::Mat& image, const std_msgs::Header& header,
        const std::string& encoding = sensor_msgs::image_encodings::RGB8) {
    cv_bridge::CvImage output_image{header, encoding, image};
    publisher.publish(output_image.toImageMsg());
}

void StereoFactorFinder::stereo_callback(const sensor_msgs::ImageConstPtr& left_image_msg,
        const sensor_msgs::ImageConstPtr& right_image_msg, const sensor_msgs::CameraInfoConstPtr& left_info,
        const sensor_msgs::CameraInfoConstPtr& right_info) {
    // Convert to OpenCV
    const cv_bridge::CvImageConstPtr left_image = cv_bridge::toCvShare(left_image_msg);
    const cv_bridge::CvImageConstPtr right_image = cv_bridge::toCvShare(right_image_msg);

    // Optional introspection arguments
    StereoFeatureTracker::Statistics stats;
    std::optional<std::reference_wrapper<StereoFeatureTracker::Statistics>> stats_ref = std::nullopt;
    if (print_stats) {
        stats_ref = stats;
    }
    StereoFeatureTracker::IntermediateImages intermediate_images;
    std::optional<std::reference_wrapper<StereoFeatureTracker::IntermediateImages>> intermediate_images_ref =
            std::nullopt;
    if (publish_intermediate_results) {
        intermediate_images.keypoint_draw_flags = keypoint_draw_flags;
        intermediate_images.match_draw_flags = match_draw_flags;
        intermediate_images_ref = intermediate_images;
    }
    
    // Run processing pipeline
    const ros::WallTime tic = ros::WallTime::now();
    auto tracked_matches = tracker->process(left_image->image, right_image->image, stats_ref, intermediate_images_ref);
    ROS_INFO_STREAM("Tracker processing completed in " << (ros::WallTime::now() - tic).toSec() << " seconds for stereo"
            " data at t = " << left_image->header.stamp);

    // Optional printing and publishing of internal information
    if (print_stats) {
        ROS_INFO_STREAM(stats.to_string());
    }
    if (publish_intermediate_results) {
        const std_msgs::Header& header = left_image->header;
        publish_image(extracted_keypoints_left_publisher, intermediate_images.extracted_keypoints[0], header);
        publish_image(extracted_keypoints_right_publisher, intermediate_images.extracted_keypoints[1], header);
        publish_image(sof_matches_left_publisher, intermediate_images.sof_matches[0], header);
        publish_image(sof_matches_right_publisher, intermediate_images.sof_matches[1], header);
        publish_image(stereo_filtered_matches_publisher, intermediate_images.stereo_filtered_matches, header);
        publish_image(new_matches_publisher, intermediate_images.new_matches, header);
        publish_image(tracked_matches_publisher, intermediate_images.tracked_matches, header);
    }

    // Publish Stereo Landmarks
    auto stereo_landmarks = boost::make_shared<serpent::StereoLandmarks>();
    stereo_landmarks->header = left_image->header;
    to_ros(stereo_landmarks->landmarks, tracked_matches);
    stereo_landmarks->left_info = *left_info;
    stereo_landmarks->right_info = *right_info;
    stereo_landmarks_publisher.publish(stereo_landmarks);

    // Keep images in scope
    previous_left_image = left_image;
    previous_right_image = right_image;
}

}
