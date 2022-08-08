#include "serpent/stereo_keypoint_matcher.hpp"
#include <opencv2/core/mat.hpp>

namespace serpent {

double sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2i& image1_top_left,
        const cv::Point2i& image2_top_left, const cv::Size& window) {
    if (image1.type() != CV_8U || image2.type() != CV_8U) {
        throw std::runtime_error("Expected images to be of type CV_8U");
    }
    double sum{0.0};
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            sum += std::abs(
                    static_cast<double>(image1.at<unsigned char>(image1_top_left.y + row, image1_top_left.x + col)) - 
                    static_cast<double>(image2.at<unsigned char>(image2_top_left.y + row, image2_top_left.x + col)));
        }
    }
    return sum;
}

StereoKeyPointMatcher::StereoKeyPointMatcher(const MatchingCostFunction cost_function, const cv::Size& window):
    cost_function(cost_function), window(window)
{
    if (window.width < 1 || window.width % 2 == 0 || window.height < 1 || window.height % 2 == 0) {
        throw std::runtime_error("Window must have positive odd-length dimensions.");
    }
}

cv::Ptr<StereoKeyPointMatcher> StereoKeyPointMatcher::create(const MatchingCostFunction cost_function,
        const cv::Size& window) {
    return cv::makePtr<StereoKeyPointMatcher>(cost_function, window);
}

cv::KeyPoint StereoKeyPointMatcher::keypoint_in_right_image(const cv::KeyPoint& left_image_keypoint,
        const cv::Mat& right_image, const cv::Mat& left_image, double& cost) const {
    // Compute the top left coordinate
    const cv::Point2i left_image_top_left{static_cast<int>(std::floor(left_image_keypoint.pt.x)) - (window.width - 1)/2,
            static_cast<int>(std::floor(left_image_keypoint.pt.y)) - (window.height - 1)/2};
    cv::Point2i right_image_top_left = left_image_top_left;

    // Check that the x and y coordinates are valid, and return early otherwise
    cost = std::numeric_limits<double>::max();
    if (right_image_top_left.x < 0 || (right_image_top_left.x + window.width) >= right_image.cols ||
            right_image_top_left.y < 0 || (right_image_top_left.y + window.height) >= right_image.rows) {
        return cv::KeyPoint{};
    }

    // Iterate over valid x coordinates
    cv::Point2i min_cost_top_left;
    while (right_image_top_left.x >= 0) {
        double window_cost = cost_function(left_image, right_image, left_image_top_left, right_image_top_left, window);
        if (window_cost < cost) {
            cost = window_cost;
            min_cost_top_left = right_image_top_left;
        }
        --right_image_top_left.x;
    }

    // Return the keypoint in the middle of the pixel
    cv::KeyPoint right_image_keypoint = left_image_keypoint;
    right_image_keypoint.pt = cv::Point2f{
            static_cast<float>(min_cost_top_left.x) + static_cast<float>(window.width - 1) / 2.f,
            static_cast<float>(min_cost_top_left.y) + static_cast<float>(window.height - 1) / 2.f};
    return right_image_keypoint;
}

std::vector<cv::KeyPoint> StereoKeyPointMatcher::keypoints_in_right_image(const std::vector<cv::KeyPoint>& left_image_keypoints,
        const cv::Mat& left_image, const cv::Mat& right_image, std::vector<double>& costs) const {
    std::vector<cv::KeyPoint> right_image_keypoints{left_image_keypoints.size()};
    costs.resize(left_image_keypoints.size());
    for (std::size_t i = 0; i < left_image_keypoints.size(); ++i) {
        right_image_keypoints[i] = keypoint_in_right_image(left_image_keypoints[i], left_image, right_image, costs[i]);
    }
    return right_image_keypoints;
}

}
