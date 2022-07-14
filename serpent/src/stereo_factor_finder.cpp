#include "serpent/stereo_factor_finder.hpp"
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

template<typename Feature2DType>
cv::Ptr<Feature2DType> create_from_params(const ros::NodeHandle& nh);

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

StereoFactorFinder::StereoFactorFinder():
    nh("~"), it(nh), stereo_sync(10)
{
    // Subscribers
    left_image_subcriber.subscribe(nh, "input/stereo/left/image", 10);
    right_image_subcriber.subscribe(nh, "input/stereo/right/image", 10);
    left_info_subcriber.subscribe(nh, "input/stereo/left/camera_info", 10);
    right_info_subcriber.subscribe(nh, "input/stereo/right/camera_info", 10);
    stereo_sync.connectInput(left_image_subcriber, right_image_subcriber, left_info_subcriber, right_info_subcriber);
    stereo_sync.registerCallback(boost::bind(&StereoFactorFinder::stereo_callback, this, _1, _2, _3, _4));

    // Publishers
    left_image_publisher = it.advertise("stereo/left/detections/image", 1);
    left_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/left/detections/camera_info", 1);
    right_image_publisher = it.advertise("stereo/right/detections/image", 1);
    right_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("stereo/right/detections/camera_info", 1);
    image_matches_publisher = it.advertise("stereo/matches/image", 1);

    // Test: SOF Publishers
    prev_left_publisher = it.advertise("stereo/left/sof_prev/image", 1);
    sof_left_publisher = it.advertise("stereo/left/sof/image", 1);
    prev_right_publisher = it.advertise("stereo/right/sof_prev/image", 1);
    sof_right_publisher = it.advertise("stereo/right/sof/image", 1);

    // Detector
    const std::string feature_type = nh.param<std::string>("stereo_factors/detector/type", "ORB");
    if (feature_type == "GFTT") {
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

    // Descriptor
    const std::string descriptor_type = nh.param<std::string>("stereo_factors/descriptor/type", "ORB");
    if (descriptor_type == "GFTT") {
        descriptor = create_from_params<cv::GFTTDetector>(nh);
    } else if (descriptor_type == "ORB") {
        descriptor = create_from_params<cv::ORB>(nh);
    } else if (descriptor_type == "SIFT") {
        #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 4
        descriptor = create_from_params<cv::SIFT>(nh);
        #else
        throw std::runtime_error("SIFT requires OpenCV >= 4.4.0. Current OpenCV version: "
                + std::to_string(CV_VERSION_MAJOR) + "." + std::to_string(CV_VERSION_MINOR) + "."
                + std::to_string(CV_VERSION_REVISION));
        #endif
    } else {
        throw std::runtime_error("Stereo factor descriptor type \'" + descriptor_type + "\' not recognised.");
    }

    // Matcher
    const std::string matcher_type = nh.param<std::string>("stereo_factors/matcher/type", "FLANN");
    if (matcher_type == "BRUTE_FORCE") {
        const cv::NormTypes norm_type = to_norm_type(
                nh.param<std::string>("stereo_factors/matcher/brute_force/norm_type", "NORM_L2"));
        const bool cross_check = nh.param<bool>("stereo_factors/matcher/brute_force/cross_check", false);
        matcher = cv::BFMatcher::create(norm_type, cross_check);
    } else if (matcher_type == "FLANN") {
        if (descriptor_type == "ORB") {
            throw std::runtime_error("Cannot use FLANN (currently) for non-float descriptors (ORB, BRIEF). Use"
                    " Brute-Force matcher with NORM_HAMMING norm_type instead, or change descriptor. TODO: implement"
                    " options for FLANN to broaden compatibility.");
        }
        // TODO: create from constructor with search and index params
        matcher = cv::FlannBasedMatcher::create();
    } else {
        throw std::runtime_error("Stereo factor matcher type \'" + matcher_type + "\' not recognised.");
    }

    // Filtering
    if (nh.param<bool>("stereo_factors/distance_match_filter/enabled", false)) {
        const float descriptor_distance_threshold = nh.param<float>(
                "stereo_factors/distance_match_filter/descriptor_distance_threshold",
                std::numeric_limits<float>::max());
        distance_filter = serpent::DistanceMatchFilter::create(descriptor_distance_threshold);
    }
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

    // Visualisation
    if (nh.param<bool>("stereo_factors/visualisation/rich_keypoints", true)) {
        draw_feature_flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    } else {
        draw_feature_flag = cv::DrawMatchesFlags::DEFAULT;
    }
}

void StereoFactorFinder::stereo_callback(const sensor_msgs::ImageConstPtr& image_left_msg,
        const sensor_msgs::ImageConstPtr& image_right_msg, const sensor_msgs::CameraInfoConstPtr& info_left,
        const sensor_msgs::CameraInfoConstPtr& info_right) {
    // Convert to OpenCV
    const cv_bridge::CvImageConstPtr image_left = cv_bridge::toCvShare(image_left_msg);
    const cv_bridge::CvImageConstPtr image_right = cv_bridge::toCvShare(image_right_msg);

    // Feature detection
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

    // Feature description
    cv::Mat descriptors_left, descriptors_right;
    tic = ros::WallTime::now();
    descriptor->compute(image_left->image, keypoints_left, descriptors_left);
    ROS_INFO_STREAM("Computing descriptors for left camera took " << (ros::WallTime::now() - tic).toSec() << " s.");
    tic = ros::WallTime::now();
    descriptor->compute(image_right->image, keypoints_right, descriptors_right);
    ROS_INFO_STREAM("Computing descriptors for right camera took " << (ros::WallTime::now() - tic).toSec() << " s.");

    // Feature matching (best match, note knn and radius are also available)
    tic = ros::WallTime::now();
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_left, descriptors_right, matches);
    ROS_INFO_STREAM("Computing matches took " << (ros::WallTime::now() - tic).toSec() << " s.");

    // Filtering
    if (distance_filter) {
        matches = distance_filter->filter(matches);
    }
    matches = stereo_filter->filter(keypoints_left, keypoints_right, matches);

    // Visualise matches
    cv_bridge::CvImage output_image_matches{image_left->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
    cv::drawMatches(image_left->image, keypoints_left, image_right->image, keypoints_right, matches,
            output_image_matches.image, cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), std::vector<char>(),
            draw_feature_flag);
    image_matches_publisher.publish(output_image_matches.toImageMsg());

    // Convert keypoints to matrix
    std::vector<cv::Point2f> points_left, points_right;
    cv::KeyPoint::convert(keypoints_left, points_left);
    cv::KeyPoint::convert(keypoints_right, points_right);
    const cv::Mat points_left_mat(points_left);
    const cv::Mat points_right_mat(points_right);

    if (prev_image_left && prev_image_right) {
        // Sparse Optical Flow
        cv::Mat sof_points_left_mat, sof_points_right_mat;
        cv::Mat sof_status_left_mat, sof_status_right_mat;
        cv::Mat sof_error_left_mat, sof_error_right_mat;
        sparse_optical_flow->calc(prev_image_left->image, image_left->image, prev_points_left_mat, sof_points_left_mat,
                sof_status_left_mat, sof_error_left_mat);
        sparse_optical_flow->calc(prev_image_right->image, image_right->image, prev_points_right_mat,
                sof_points_right_mat, sof_status_right_mat, sof_error_right_mat);

        // Convert matrix to keypoints
        std::vector<cv::Point2f> sof_points_left, sof_points_right;
        for (std::size_t i = 0; i < sof_points_left_mat.rows; ++i) {
            if (sof_status_left_mat.at<unsigned char>(i) == 1) {
                sof_points_left.push_back(sof_points_left_mat.at<cv::Point2f>(i));
            }
        }
        ROS_INFO_STREAM(sof_points_left.size() << " features retained by SOF in left image");
        for (std::size_t i = 0; i < sof_points_right_mat.rows; ++i) {
            if (sof_status_right_mat.at<unsigned char>(i) == 1) {
                sof_points_right.push_back(sof_points_right_mat.at<cv::Point2f>(i));
            }
        }
        ROS_INFO_STREAM(sof_points_right.size() << " features retained by SOF in right image");
        std::vector<cv::KeyPoint> sof_keypoints_left, sof_keypoints_right;
        cv::KeyPoint::convert(sof_points_left, sof_keypoints_left);
        cv::KeyPoint::convert(sof_points_right, sof_keypoints_right);

        // Test: Publish SOF images and previous frames
        cv_bridge::CvImage sof_image_left{image_left->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
        cv_bridge::CvImage sof_image_right{image_right->header, sensor_msgs::image_encodings::TYPE_8UC3, cv::Mat()};
        cv::drawKeypoints(image_left->image, sof_keypoints_left, sof_image_left.image, cv::Scalar::all(-1),
                draw_feature_flag);
        cv::drawKeypoints(image_right->image, sof_keypoints_right, sof_image_right.image, cv::Scalar::all(-1),
                draw_feature_flag);
        sof_left_publisher.publish(sof_image_left.toImageMsg());
        sof_right_publisher.publish(sof_image_right.toImageMsg());
        prev_left_publisher.publish(prev_output_image_left.toImageMsg());
        prev_right_publisher.publish(prev_output_image_right.toImageMsg());
    }
    prev_image_left = image_left;
    prev_image_right = image_right;
    prev_points_left_mat = points_left_mat;
    prev_points_right_mat = points_right_mat;

    // Test: previous output images
    prev_output_image_left = output_image_left;
    prev_output_image_right = output_image_right;
}

}
