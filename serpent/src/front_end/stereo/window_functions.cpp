#include "serpent/front_end/stereo/window_functions.hpp"

#include <opencv2/core/mat.hpp>

namespace serpent {

bool window_within_image(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window) {
    return top_left.x >= 0 && (top_left.x + window.width) <= image.cols && top_left.y >= 0 &&
           (top_left.y + window.height) <= image.rows;
}

double window_sum(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window) {
    double sum;
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            sum += static_cast<double>(image.at<unsigned char>(top_left.y + row, top_left.x + col));
        }
    }
    return sum;
}

double window_mean(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window) {
    return window_sum(image, top_left, window) / static_cast<double>(window.area());
}

double window_sum_squares(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window) {
    double squares;
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            squares += std::pow(static_cast<double>(image.at<unsigned char>(top_left.y + row, top_left.x + col)), 2.0);
        }
    }
    return squares;
}

}
