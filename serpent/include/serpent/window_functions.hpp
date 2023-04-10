#ifndef SERPENT_WINDOW_FUNCTIONS_HPP
#define SERPENT_WINDOW_FUNCTIONS_HPP

#include <opencv2/core/types.hpp>

namespace serpent {

bool window_within_image(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window);

double window_sum(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window);

double window_mean(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window);

double window_sum_squares(const cv::Mat& image, const cv::Point2i& top_left, const cv::Size& window);

}

#endif
