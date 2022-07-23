#ifndef SERPENT_STEREO_FEATURE_TRACKER_HPP
#define SERPENT_STEREO_FEATURE_TRACKER_HPP

#include "serpent/match_filters.hpp"
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
        std::array<std::size_t, 2> extracted_kp_count;
        std::array<std::size_t, 2> tracked_kp_count;
        std::size_t hypothesis_match_count;
        std::size_t hypothesis_match_id_count;
        std::array<std::size_t, 2> filtered_extracted_kp_count;
        std::array<std::size_t, 2> merged_kp_count;
        std::size_t match_count;
        std::size_t distance_filtered_match_count;
        std::size_t stereo_filtered_match_count;
        std::size_t consistent_match_count;
        std::size_t consistent_match_id_count;
        std::size_t consistent_tracked_matches;
        std::size_t consistent_new_matches;
        std::array<std::size_t, 2> consistent_match_kp_count;
        int longest_tracked_match_id;

        std::string to_string() const;
    };

    struct IntermediateImages {
        LRImages extracted_keypoints;
        std::array<cv::Mat, 2> sof_matches;
        LRImages merged_keypoints;
        cv::Mat raw_matches;
        cv::Mat distance_filtered_matches;
        cv::Mat stereo_filtered_matches;
        cv::Mat consistent_new_matches;
        cv::Mat consistent_tracked_matches;

        cv::Scalar keypoint_colours = cv::Scalar::all(-1);
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

    explicit StereoFeatureTracker(const cv::Ptr<cv::Feature2D> detector, const cv::Ptr<cv::Feature2D> descriptor,
            const cv::Ptr<cv::DescriptorMatcher> matcher, const cv::Ptr<cv::SparseOpticalFlow> sof,
            const cv::Ptr<serpent::StereoMatchFilter> stereo_filter,
            const cv::Ptr<serpent::DistanceMatchFilter> distance_filter, const float new_feature_dist_threshold);

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
    void append_keypoints(const LRKeyPoints& keypoints, LRKeyPoints& new_track_hypotheses) const;

    /**
     * @brief Here we rely on the assumption that keypoints with the same index up to the size of match_hypothesis_ids
     * were matches in the previous frame, saving unnecessary brute force computation.
     * 
     * @param matches 
     * @param match_hypotheses 
     * @param match_hypothesis_ids 
     * @param consistent_matches 
     * @param consistent_match_ids 
     * @return std::size_t 
     */
    std::size_t extract_consistent_matches(const LRMatches& matches, const LRMatchIds& match_hypothesis_ids,
            LRMatches& consistent_matches, LRMatchIds& consistent_match_ids);

    LRKeyPoints extract_keypoints(const LRImages& images) const;

    /**
     * @brief Extract just the keypoints present in the matches, and update the match indices.
     * 
     * @param keypoints 
     * @param matches 
     * @return LRKeyPoints 
     */
    LRKeyPoints extract_matched_keypoints(const LRKeyPoints& keypoints, LRMatches& matches);

    LRMatches match_tracked_keypoints(const LRImages& images, LRKeyPoints& keypoints) const;

    LRKeyPoints remove_already_tracked_keypoints(const LRKeyPoints& keypoints,
            const LRKeyPoints& tracked_keypoints) const;

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
    cv::Ptr<cv::Feature2D> descriptor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Ptr<cv::SparseOpticalFlow> sof;
    cv::Ptr<StereoMatchFilter> stereo_filter;
    cv::Ptr<DistanceMatchFilter> distance_filter;

    //// Configuration
    const float new_feature_sqr_dist_threshold;

    //// State
    int frame_number;
    int next_match_id;
    LRImages previous_images;
    LRKeyPointMatches previous_track_data;
};

}

#endif
