#ifndef SERPENT_STEREO_KEYPOINT_MATCHER_HPP
#define SERPENT_STEREO_KEYPOINT_MATCHER_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

#include "serpent/stereo_cost_functions.hpp"

namespace serpent {

/**
 * @brief Match features from one stereo frame to the other using a windowed stereo cost match function.
 *
 */
class StereoKeyPointMatcher {
public:
    /**
     * @brief Matching filter configuration.
     *  * UNIDIRECTIONAL = one-way matching, where a keypoint is matched to the min-cost of valid keypoints.
     *  * BIDIRECTIONAL = two-way matching, where a keypoint is matched if left-to-right and right-to-left yield the
     * same match.
     *  * RATIO_TEST = one-way matching, where a keypoint is matched to the min-cost of valid keypoints if the ratio of
     * its cost to the cost of the second-best keypoint is less than a ratio, or there is only one valid keypoint.
     */
    enum class MatchingFilter { UNIDIRECTIONAL, BIDIRECTIONAL, RATIO_TEST };

    /**
     * @brief Construct a stereo key point matcher.
     *
     * @param window dimensions of the stereo matching window
     * @param vertical_pixel_threshold number of pixels above or below the epipolar line to accept a keypoint match.
     * @param matching_filter matching filter (see MatchingFilter)
     * @param ratio ratio for the RATIO_TEST matching_filter
     */
    explicit StereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Scan along the epipolar line (left or right of the horizontal coordinate) in the other image, and return
     * the keypoint with the minimum matching cost. Also return the cost.
     *
     * @param image_keypoint keypoint in left image if find_in_right is true, otherwise keypoint in right image
     * @param left_image
     * @param right_image
     * @param find_in_right true to search right image, false to search left image
     * @param cost
     * @return cv::KeyPoint
     */
    virtual cv::KeyPoint keypoint_in_other_image(const cv::KeyPoint& image_keypoint, const cv::Mat& left_image,
            const cv::Mat& right_image, const bool find_in_right, double& cost, bool& matched) const;

    /**
     * @brief  Iterate over a set of image keypoints, and return the index of the keypoint in the other image that has
     * the lowest matching cost, such that it is within a vertical pixel threshold and is to the left of the left frame
     * keypoint if searaching right, or to the right of right frame keypoint if searching left. Also return the cost.
     *
     * Complexity is O(m*C), where m is the number of image keypoints, C is the complexity of the matching cost
     * function.
     *
     * UNIDIRECTIONAL and RATIO_TEST matching filters are handled in this function. BIDIRECTIONAL can only be handled in
     * keypoint_indices_in_other_image().
     *
     * @param image_keypoint left image keypoint if find_in_right is true, otherwise right image keypoints
     * @param left_image
     * @param right_image
     * @param other_image_keypoints right image keypoints if find_in_right is true, otherwise left image keypoints
     * @param vertical_pixel_threshold
     * @param find_in_right
     * @param cost
     * @return int index of right image keypoint. -1 if unmatched
     */
    virtual int keypoint_index_in_other_image(const cv::KeyPoint& image_keypoint,
            const std::vector<cv::KeyPoint>& other_image_keypoints, const cv::Mat& left_image,
            const cv::Mat& right_image, const bool find_in_right, double& cost) const;

    /**
     * @brief Find the lowest cost match for a vector of keypoints, and return these costs.
     *
     * @param image_keypoints (from left frame if find_in_right is true, otherwise from right frmae)
     * @param left_image
     * @param right_image
     * @param find_in_right
     * @param costs
     * @param matched for each keypoint in image_keypoints, true if matched, false otherwise
     * @return std::vector<cv::KeyPoint>
     */
    std::vector<cv::KeyPoint> keypoints_in_other_image(const std::vector<cv::KeyPoint>& image_keypoints,
            const cv::Mat& left_image, const cv::Mat& right_image, const bool find_in_right, std::vector<double>& costs,
            std::vector<bool>& matched) const;

    /**
     * @brief Find the lowest cost match for a vector of image keypoints. See keypoint_index_in_other_image().
     *
     * Complexity is O(n*m*C), where n is the number of left image keypoints, m is the number of right image
     * keypoints, C is the complexity of the matching cost function.
     *
     * This function handles all matching filters: UNIDIRECTIONAL, BIDIRECTIONAL, and RATIO_TEST.
     *
     * @param left_image_keypoints
     * @param left_image
     * @param right_image
     * @param right_image_keypoints
     * @param vertical_pixel_threshold
     * @param find_in_right
     * @param costs
     * @return std::vector<int> index of right image keypoint which matches each left_image_keypoint. -1 if unmatched.
     */
    std::vector<int> keypoint_indices_in_other_image(const std::vector<cv::KeyPoint>& left_image_keypoints,
            const std::vector<cv::KeyPoint>& right_image_keypoints, const cv::Mat& left_image,
            const cv::Mat& right_image, const bool find_in_right, std::vector<double>& costs) const;

protected:
    virtual MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const = 0;

    cv::Point2i top_left(const cv::Point2f& centre) const;

    //// Configuration
    cv::Size window;
    float vertical_pixel_threshold;
    MatchingFilter matching_filter;
    double ratio;

    // Pre-compute the size of the half-window (rounded down) to avoid repetitive computation. Equals [(w-1)/2, (h-1)/2]
    cv::Size half_window_floor;
    cv::Point2f half_window;
};

/**
 * @brief Convert string to StereoKeyPointMatcher::MatchingFilter.
 *
 * @param matching_filter
 * @return StereoKeyPointMatcher::MatchingFilter
 */
StereoKeyPointMatcher::MatchingFilter to_matching_filter(const std::string& matching_filter);

/**
 * @brief Stereo keypoint matcher using the sum of absolute differences (SAD) cost function.
 *
 */
class SADStereoKeyPointMatcher : public StereoKeyPointMatcher {
public:
    explicit SADStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     *
     * @param cost_function
     * @param window
     * @return cv::Ptr<SADStereoKeyPointMatcher>
     */
    static cv::Ptr<SADStereoKeyPointMatcher> create(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

protected:
    /**
     * @brief Create a cost function for the class.
     *
     * @param reference_image
     * @param reference_top_left
     * @return MatchingCostFunction
     */
    MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const override;
};

/**
 * @brief Stereo keypoint matcher using the sum of squared differences (SSD) cost function.
 *
 */
class SSDStereoKeyPointMatcher : public StereoKeyPointMatcher {
public:
    explicit SSDStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     *
     * @param cost_function
     * @param window
     * @return cv::Ptr<SSDStereoKeyPointMatcher>
     */
    static cv::Ptr<SSDStereoKeyPointMatcher> create(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

protected:
    /**
     * @brief Create a cost function for the class.
     *
     * @param reference_image
     * @param reference_top_left
     * @return MatchingCostFunction
     */
    MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const override;
};

/**
 * @brief Stereo keypoint matcher using the zero mean sum of absolute differences (SAD) cost function.
 *
 */
class ZeroMeanSADStereoKeyPointMatcher : public StereoKeyPointMatcher {
public:
    explicit ZeroMeanSADStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     *
     * @param cost_function
     * @param window
     * @return cv::Ptr<ZeroMeanSADStereoKeyPointMatcher>
     */
    static cv::Ptr<ZeroMeanSADStereoKeyPointMatcher> create(const cv::Size& window,
            const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

protected:
    /**
     * @brief Create a cost function for the class.
     *
     * @param reference_image
     * @param reference_top_left
     * @return MatchingCostFunction
     */
    MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const override;
};

/**
 * @brief Stereo keypoint matcher using the locally scaled sum of absolute differences (SAD) cost function.
 *
 */
class LocallyScaledSADStereoKeyPointMatcher : public StereoKeyPointMatcher {
public:
    explicit LocallyScaledSADStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     *
     * @param cost_function
     * @param window
     * @return cv::Ptr<LocallyScaledSADStereoKeyPointMatcher>
     */
    static cv::Ptr<LocallyScaledSADStereoKeyPointMatcher> create(const cv::Size& window,
            const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

protected:
    /**
     * @brief Create a cost function for the class.
     *
     * @param reference_image
     * @param reference_top_left
     * @return MatchingCostFunction
     */
    MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const override;
};

/**
 * @brief Stereo keypoint matcher using the normalised cross-correlation (NCC) objective function (cost function is its
 * negative).
 *
 */
class NCCStereoKeyPointMatcher : public StereoKeyPointMatcher {
public:
    explicit NCCStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     *
     * @param cost_function
     * @param window
     * @return cv::Ptr<NCCStereoKeyPointMatcher>
     */
    static cv::Ptr<NCCStereoKeyPointMatcher> create(const cv::Size& window, const float vertical_pixel_threshold = 1.0,
            const MatchingFilter matching_filter = MatchingFilter::UNIDIRECTIONAL, const double ratio = 0.5);

protected:
    /**
     * @brief Create a cost function for the class.
     *
     * @param reference_image
     * @param reference_top_left
     * @return MatchingCostFunction
     */
    MatchingCostFunction generate_cost_function(const cv::Mat& reference_image,
            const cv::Point2i& reference_top_left) const override;
};

}

#endif
