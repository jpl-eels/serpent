#ifndef SERPENT_STEREO_FEATURE_TRACKER_HPP
#define SERPENT_STEREO_FEATURE_TRACKER_HPP

#include "serpent/match_filters.hpp"
#include "serpent/stereo_keypoint_matcher.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace serpent {

bool approximately_near(const cv::Point2f& p1, const cv::Point2f& p2, const float sqr_dist);

bool approximately_near(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const float sqr_dist);

class StereoFeatureTracker {
public:
    using LRImages = std::array<cv::Mat, 2>;
    using LRKeyPoints = std::array<std::vector<cv::KeyPoint>, 2>;
    using LRDescriptors = std::array<cv::Mat, 2>;
    using LRMatches = std::vector<cv::DMatch>;
    using LRMatchIds = std::vector<int>;
    using LRF2FMatches = std::array<std::vector<cv::DMatch>, 2>;

    struct Statistics {
        int frame_number;
        int max_match_id;
        int longest_tracked_match_id;
        std::size_t extracted_kp_count;
        std::array<std::size_t, 2> tracked_kp_count;
        std::size_t hypothesis_match_count;
        std::size_t hypothesis_match_id_count;
        std::size_t filtered_extracted_kp_count;
        std::size_t tracked_match_count;
        std::size_t new_match_count;
        std::size_t total_match_count;
        std::size_t total_match_id_count;

        std::string to_string() const;
    };

    struct IntermediateImages {
        cv::Mat extracted_keypoints;
        std::array<cv::Mat, 2> sof_matches;
        cv::Mat stereo_filtered_matches;
        cv::Mat new_matches;
        cv::Mat tracked_matches;

        cv::Scalar keypoint_colours = cv::Scalar::all(-1);
        cv::Scalar roi_colour = cv::Scalar(0, 255, 0);
        int roi_thickness = 2;
        cv::Scalar new_match_colour = cv::Scalar(0, 255, 0);
        cv::Scalar tracked_match_colour = cv::Scalar(0, 0, 255);
        cv::Scalar negative_match_colour = cv::Scalar(255, 0, 0);
        cv::DrawMatchesFlags keypoint_draw_flags = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
        cv::DrawMatchesFlags match_draw_flags = cv::DrawMatchesFlags::DEFAULT;
    };

    struct LRKeyPointMatches {
        LRKeyPoints keypoints;
        LRMatches matches;
        LRMatchIds match_ids;
    };

    explicit StereoFeatureTracker(const cv::Ptr<cv::Feature2D> detector, const cv::Ptr<cv::SparseOpticalFlow> sof,
            const cv::Ptr<serpent::StereoMatchFilter> stereo_filter, cv::Ptr<StereoKeyPointMatcher> stereo_matcher,
            const float new_feature_dist_threshold, const double stereo_match_cost_threshold,
            const cv::Rect2i& roi = cv::Rect2i{});

    /**
     * @brief Process a new left-right stereo pair, and return the tracked features in the new frame pair.
     * 
     * @param left_image 
     * @param right_image 
     * @return LRKeyPointMatches
     */
    LRKeyPointMatches process(const cv::Mat& left_image, const cv::Mat& right_image,
            std::optional<std::reference_wrapper<Statistics>> stats = std::nullopt, 
            std::optional<std::reference_wrapper<IntermediateImages>> intermediate_images = std::nullopt);

private:
    /**
     * @brief Extract keypoints from an image using the detector.
     * 
     * @param image image for feature detection
     * @param roi region of interest to restrict feature detection
     * @return std::vector<cv::KeyPoint> 
     */
    std::vector<cv::KeyPoint> extract_keypoints(const cv::Mat& image, cv::InputArray roi = cv::noArray()) const;

    /**
     * @brief Extract just the keypoints present in the matches, and update the match indices.
     * 
     * @param keypoints 
     * @param matches 
     * @return LRKeyPoints 
     */
    LRKeyPoints extract_matched_keypoints(const LRKeyPoints& keypoints, LRMatches& matches) const;

    /**
     * @brief Extract the match ids by index.
     * 
     * @param match_ids 
     * @param indices 
     * @return LRMatchIds 
     */
    LRMatchIds extract_match_ids(const LRMatchIds& match_ids, const std::vector<std::size_t>& indices) const;

    /**
     * @brief Remove new stereo keypoint pairs if their cost (computed by the matching algorithm) exceeds a threshold.
     * 
     * @param new_keypoints 
     * @param costs stereo match costs, to be compared against the threshold
     * @return LRKeyPoints 
     */
    LRKeyPoints filter_new_stereo_keypoints(const LRKeyPoints& new_keypoints, const std::vector<double>& costs) const;

    /**
     * @brief Add new keypoints to track data, adding a match with a new match id for each one.
     * 
     * @param new_keypoints 
     * @param track_data 
     */
    void merge_new_stereo_keypoints(const LRKeyPoints& new_keypoints, LRKeyPointMatches& track_data);

    /**
     * @brief Return a filtered set of keypoints which are not too close to any reference keypoints, according to a
     * Euclidean pixel distance threshold set upon construction of the class.
     * 
     * @param keypoints 
     * @param reference_keypoints 
     * @return std::vector<cv::KeyPoint> 
     */
    std::vector<cv::KeyPoint> remove_close_keypoints(const std::vector<cv::KeyPoint>& keypoints,
            const std::vector<cv::KeyPoint>& reference_keypoints) const;

    /**
     * @brief Track keypoints from previous images. Only if the keypoints of a match are tracked successfully in both
     * frames are they kept. Even though the matches are set, they could also be inferred, since keypoints with the same
     * index were matched previously.
     * 
     * @param images 
     * @param f2f_matches 
     * @return LRKeyPointMatches 
     */
    LRKeyPointMatches track_previous_keypoints(const LRImages& images, LRKeyPoints& all_sof_keypoints,
            LRF2FMatches& f2f_matches) const;

    //// Algorithms
    cv::Ptr<cv::Feature2D> detector;
    cv::Ptr<cv::SparseOpticalFlow> sof;
    cv::Ptr<StereoMatchFilter> stereo_filter;
    cv::Ptr<StereoKeyPointMatcher> stereo_matcher;

    //// Configuration
    const float new_feature_sqr_dist_threshold;
    const double stereo_match_cost_threshold;
    const cv::Rect2i roi;
    cv::Mat roi_mask;

    //// State
    int frame_number;
    int next_match_id;
    LRImages previous_images;
    LRKeyPointMatches previous_track_data;
};

}

#endif
