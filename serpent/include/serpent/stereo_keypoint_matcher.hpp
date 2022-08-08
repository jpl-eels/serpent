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

    explicit StereoKeyPointMatcher(const MatchingCostFunction cost_function, const cv::Size& window);

    /**
     * @brief Create an instance of the class in the OpenCV style.
     * 
     * @param cost_function 
     * @param window 
     * @return cv::Ptr<StereoKeyPointMatcher> 
     */
    static cv::Ptr<StereoKeyPointMatcher> create(const MatchingCostFunction cost_function, const cv::Size& window);

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

private:
    MatchingCostFunction cost_function;
    cv::Size window;
};

}

#endif
