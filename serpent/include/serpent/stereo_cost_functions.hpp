#ifndef SERPENT_STEREO_COST_FUNCTIONS_HPP
#define SERPENT_STEREO_COST_FUNCTIONS_HPP

#include <opencv2/core/types.hpp>
#include <vector>

namespace serpent {

using MatchingCostFunctionPtr = double (*)(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&,
        const cv::Size&);
using MatchingCostFunction =
        std::function<double(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&, const cv::Size&)>;

double sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2i& image1_top_left,
        const cv::Point2i& image2_top_left, const cv::Size& window);

double sum_of_squared_differences(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2i& image1_top_left,
        const cv::Point2i& image2_top_left, const cv::Size& window);

double zero_mean_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window);

double zero_mean_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double mean1);

double locally_scaled_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window);

double locally_scaled_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double mean1);

double negative_normalised_cross_correlation(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window);

double negative_normalised_cross_correlation(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double sum_squares1);

}

#endif
