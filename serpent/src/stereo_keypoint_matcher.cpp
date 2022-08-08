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

StereoKeyPointMatcher::StereoKeyPointMatcher(const MatchingCostFunction cost_function, const cv::Size& window,
        const float vertical_pixel_threshold):
    cost_function(cost_function), window(window), vertical_pixel_threshold(vertical_pixel_threshold)
{
    if (window.width < 1 || window.width % 2 == 0 || window.height < 1 || window.height % 2 == 0) {
        throw std::runtime_error("Window must have positive odd-length dimensions.");
    }
    half_window_floor = cv::Size((window.width - 1)/2, (window.height - 1)/2);
}

cv::Ptr<StereoKeyPointMatcher> StereoKeyPointMatcher::create(const MatchingCostFunction cost_function,
        const cv::Size& window, const float vertical_pixel_threshold) {
    return cv::makePtr<StereoKeyPointMatcher>(cost_function, window, vertical_pixel_threshold);
}

cv::KeyPoint StereoKeyPointMatcher::keypoint_in_right_image(const cv::KeyPoint& left_image_keypoint,
        const cv::Mat& right_image, const cv::Mat& left_image, double& cost) const {
    // Compute the top left coordinate
    const cv::Point2i left_image_top_left = top_left(left_image_keypoint.pt);
    cv::Point2i right_image_top_left = left_image_top_left;

    // Check that the x and y coordinates are valid, and return early otherwise
    cost = std::numeric_limits<double>::max();
    if (!window_within_image(right_image_top_left, right_image)) {
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
    right_image_keypoint.pt = cv::Point2f{static_cast<float>(min_cost_top_left.x + half_window_floor.width),
            static_cast<float>(min_cost_top_left.y + half_window_floor.height)};
    return right_image_keypoint;
}

int StereoKeyPointMatcher::keypoint_index_in_right_image(const cv::KeyPoint& left_image_keypoint,
        const cv::Mat& left_image, const cv::Mat& right_image, const std::vector<cv::KeyPoint>& right_image_keypoints,
        double& cost) const {
    // Compute the top left coordinate
    const cv::Point2i left_image_top_left = top_left(left_image_keypoint.pt);

    // Find the min cost right image keypoint
    int min_cost_index{-1};
    cost = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(right_image_keypoints.size()); ++i) {
        const cv::KeyPoint& right_image_keypoint = right_image_keypoints[i];
        // Check keypoint is within vertical pixel threshold
        if (std::abs(right_image_keypoint.pt.y - left_image_keypoint.pt.y) <= vertical_pixel_threshold &&
                right_image_keypoint.pt.x <= left_image_keypoint.pt.x) {
            const cv::Point2i right_image_top_left = top_left(right_image_keypoint.pt);
            // Check window is within image
            if (window_within_image(right_image_top_left, right_image)){
                double window_cost = cost_function(left_image, right_image, left_image_top_left, right_image_top_left,
                        window);
                if (window_cost < cost) {
                    cost = window_cost;
                    min_cost_index = i;
                }
            }
        }
    }
    return min_cost_index;
}

std::vector<cv::KeyPoint> StereoKeyPointMatcher::keypoints_in_right_image(
        const std::vector<cv::KeyPoint>& left_image_keypoints, const cv::Mat& left_image, const cv::Mat& right_image,
        std::vector<double>& costs) const {
    std::vector<cv::KeyPoint> right_image_keypoints{left_image_keypoints.size()};
    costs.resize(left_image_keypoints.size());
    for (std::size_t i = 0; i < left_image_keypoints.size(); ++i) {
        right_image_keypoints[i] = keypoint_in_right_image(left_image_keypoints[i], left_image, right_image, costs[i]);
    }
    return right_image_keypoints;
}

std::vector<int> StereoKeyPointMatcher::keypoint_indices_in_right_image(
        const std::vector<cv::KeyPoint>& left_image_keypoints, const cv::Mat& left_image, const cv::Mat& right_image,
        const std::vector<cv::KeyPoint>& right_image_keypoints, std::vector<double>& costs) const {
    std::vector<int> right_image_keypoint_indices(left_image_keypoints.size());
    costs.resize(left_image_keypoints.size());
    for (std::size_t i = 0; i < left_image_keypoints.size(); ++i) {
        right_image_keypoint_indices[i] = keypoint_index_in_right_image(left_image_keypoints[i], left_image,
                right_image, right_image_keypoints, costs[i]);
    }
    return right_image_keypoint_indices;
}

cv::Point2i StereoKeyPointMatcher::top_left(const cv::Point2f& centre) const {
    return cv::Point2i{static_cast<int>(std::floor(centre.x)) - half_window_floor.width,
            static_cast<int>(std::floor(centre.y)) - half_window_floor.height};
}

bool StereoKeyPointMatcher::window_within_image(const cv::Point2i& top_left_pixel, const cv::Mat& image) const {
    return top_left_pixel.x >= 0 && (top_left_pixel.x + window.width) <= image.cols &&
            top_left_pixel.y >= 0 && (top_left_pixel.y + window.height) <= image.rows;
}

}
