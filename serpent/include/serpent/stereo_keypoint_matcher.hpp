#ifndef SERPENT_STEREO_KEYPOINT_MATCHER_HPP
#define SERPENT_STEREO_KEYPOINT_MATCHER_HPP

#include <opencv2/core/types.hpp>
#include <vector>

namespace serpent {

double sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2i& image1_top_left,
        const cv::Point2i& image2_top_left, const cv::Size& window);

/**
 * @brief Match features from one stereo frame to the other using a windowed stereo cost match function.
 * 
 * TODO: implement for the basic case of querying an integer point.
 * 
 */
class StereoKeyPointMatcher {
public:
    using MatchingCostFunction = double (*)(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&,
            const cv::Size&);

    explicit StereoKeyPointMatcher(const MatchingCostFunction cost_function, const cv::Size& window,
            const float vertical_pixel_threshold = 1.0);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     * 
     * @param cost_function 
     * @param window 
     * @return cv::Ptr<StereoKeyPointMatcher> 
     */
    static cv::Ptr<StereoKeyPointMatcher> create(const MatchingCostFunction cost_function, const cv::Size& window,
            const float vertical_pixel_threshold = 1.0);

    /**
     * @brief Scan along the epipolar line (left of the horizontal coordinate) in the right image, and return the
     * keypoint with the minimum matching cost. Also return the cost.
     * 
     * @param left_image_keypoint 
     * @param left_image 
     * @param right_image 
     * @param cost 
     * @return cv::KeyPoint 
     */
    cv::KeyPoint keypoint_in_right_image(const cv::KeyPoint& left_image_keypoint, const cv::Mat& left_image,
            const cv::Mat& right_image, double& cost) const;

    /**
     * @brief  Iterate over a set of right image keypoints, and return the index of the keypoint that has the lowest
     * matching cost to a left image keypoint, such that it is within a vertical pixel threshold of the left image
     * keypoint, and is to the left of the left frame keypoint. Also return the cost.
     * 
     * Complexity is O(m*C), where m is the number of right image keypoints, C is the complexity of the matching cost 
     * function.
     * 
     * @param left_image_keypoint 
     * @param left_image 
     * @param right_image 
     * @param right_image_keypoints 
     * @param vertical_pixel_threshold 
     * @param cost 
     * @return int index of right image keypoint. -1 if unmatched
     */
    int keypoint_index_in_right_image(const cv::KeyPoint& left_image_keypoint, const cv::Mat& left_image,
            const cv::Mat& right_image, const std::vector<cv::KeyPoint>& right_image_keypoints, double& cost) const;

    /**
     * @brief Find the lowest cost match for a vector of keypoints, and return these costs.
     * 
     * @param left_image_keypoints 
     * @param left_image 
     * @param right_image 
     * @param costs 
     * @return std::vector<cv::KeyPoint> 
     */
    std::vector<cv::KeyPoint> keypoints_in_right_image(const std::vector<cv::KeyPoint>& left_image_keypoints,
            const cv::Mat& left_image, const cv::Mat& right_image, std::vector<double>& costs) const;

    /**
     * @brief Find the lowest cost match for a vector of left image keypoints. See keypoint_index_in_right_image().
     * 
     * Complexity is O(n*m*C), where n is the number of left image keypoints, m is the number of right image
     * keypoints, C is the complexity of the matching cost function.
     * 
     * @param left_image_keypoints 
     * @param left_image 
     * @param right_image 
     * @param right_image_keypoints 
     * @param vertical_pixel_threshold 
     * @param costs 
     * @return std::vector<int> index of right image keypoint which matches each left_image_keypoint. -1 if unmatched.
     */
    std::vector<int> keypoint_indices_in_right_image(const std::vector<cv::KeyPoint>& left_image_keypoints,
            const cv::Mat& left_image, const cv::Mat& right_image,
            const std::vector<cv::KeyPoint>& right_image_keypoints, std::vector<double>& costs) const;

private:
    cv::Point2i top_left(const cv::Point2f& centre) const;

    bool window_within_image(const cv::Point2i& top_left_pixel, const cv::Mat& image) const;

    //// Configuration
    MatchingCostFunction cost_function;
    cv::Size window;
    float vertical_pixel_threshold;

    // Pre-compute the size of the half-window (rounded down) to avoid repetitive computation. Equals [(w-1)/2, (h-1)/2]
    cv::Size half_window_floor;
};

}

#endif
