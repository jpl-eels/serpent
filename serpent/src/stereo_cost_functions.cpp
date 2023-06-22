#include "serpent/stereo_cost_functions.hpp"

#include <opencv2/core/mat.hpp>
#include <utility>

#include "serpent/window_functions.hpp"

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

double sum_of_squared_differences(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2i& image1_top_left,
        const cv::Point2i& image2_top_left, const cv::Size& window) {
    if (image1.type() != CV_8U || image2.type() != CV_8U) {
        throw std::runtime_error("Expected images to be of type CV_8U");
    }
    double sum{0.0};
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            sum += std::pow(
                    static_cast<double>(image1.at<unsigned char>(image1_top_left.y + row, image1_top_left.x + col)) -
                            static_cast<double>(
                                    image2.at<unsigned char>(image2_top_left.y + row, image2_top_left.x + col)),
                    2.0);
        }
    }
    return sum;
}

double zero_mean_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window) {
    if (image1.type() != CV_8U) {
        throw std::runtime_error("Expected image1 to be of type CV_8U");
    }

    // Compute window mean
    const double mean1 = window_mean(image1, image1_top_left, window);

    // Compute sum
    return zero_mean_sum_of_absolute_differences(image1, image2, image1_top_left, image2_top_left, window, mean1);
}

double zero_mean_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double mean1) {
    if (image1.type() != CV_8U || image2.type() != CV_8U) {
        throw std::runtime_error("Expected images to be of type CV_8U");
    }

    // Compute window mean
    const double mean2 = window_mean(image2, image2_top_left, window);

    // Compute sum
    double sum{0.0};
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            sum += std::abs(
                    static_cast<double>(image1.at<unsigned char>(image1_top_left.y + row, image1_top_left.x + col)) -
                    static_cast<double>(image2.at<unsigned char>(image2_top_left.y + row, image2_top_left.x + col)) -
                    mean1 + mean2);
        }
    }
    return sum;
}

double locally_scaled_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window) {
    if (image1.type() != CV_8U) {
        throw std::runtime_error("Expected image1 to be of type CV_8U");
    }

    // Compute window mean
    const double mean1 = window_mean(image1, image1_top_left, window);

    // Compute sum
    return locally_scaled_sum_of_absolute_differences(image1, image2, image1_top_left, image2_top_left, window, mean1);
}

double locally_scaled_sum_of_absolute_differences(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double mean1) {
    if (image1.type() != CV_8U || image2.type() != CV_8U) {
        throw std::runtime_error("Expected images to be of type CV_8U");
    }

    // Compute window mean ratio
    const double mean2 = window_mean(image2, image2_top_left, window);
    const double mean_ratio = mean1 / mean2;

    // Compute sum
    double sum{0.0};
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            sum += std::abs(
                    static_cast<double>(image1.at<unsigned char>(image1_top_left.y + row, image1_top_left.x + col)) -
                    mean_ratio * static_cast<double>(
                                         image2.at<unsigned char>(image2_top_left.y + row, image2_top_left.x + col)));
        }
    }
    return sum;
}

double negative_normalised_cross_correlation(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window) {
    if (image1.type() != CV_8U) {
        throw std::runtime_error("Expected image1 to be of type CV_8U");
    }

    // Compute window squares
    const double sum_squares1 = window_sum_squares(image1, image1_top_left, window);

    // Return negative of NNC
    return negative_normalised_cross_correlation(
            image1, image2, image1_top_left, image2_top_left, window, sum_squares1);
}

double negative_normalised_cross_correlation(const cv::Mat& image1, const cv::Mat& image2,
        const cv::Point2i& image1_top_left, const cv::Point2i& image2_top_left, const cv::Size& window,
        const double sum_squares1) {
    if (image1.type() != CV_8U || image2.type() != CV_8U) {
        throw std::runtime_error("Expected images to be of type CV_8U");
    }

    // Compute window squares
    const double sum_squares2 = window_sum_squares(image2, image2_top_left, window);

    // Compute cross-correlation
    double cc{0.0};
    for (int row = 0; row < window.height; ++row) {
        for (int col = 0; col < window.width; ++col) {
            cc += static_cast<double>(image1.at<unsigned char>(image1_top_left.y + row, image1_top_left.x + col)) *
                  static_cast<double>(image2.at<unsigned char>(image2_top_left.y + row, image2_top_left.x + col));
        }
    }

    // Normalise
    const double ncc = cc / std::sqrt(sum_squares1 * sum_squares2);

    // Return negative of NNC
    return -ncc;
}

}
