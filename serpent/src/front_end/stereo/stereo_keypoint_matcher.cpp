#include "serpent/front_end/stereo/stereo_keypoint_matcher.hpp"

#include "serpent/front_end/stereo/window_functions.hpp"

namespace serpent {

StereoKeyPointMatcher::MatchingFilter to_matching_filter(const std::string& matching_filter) {
    if (matching_filter == "UNIDIRECTIONAL") {
        return StereoKeyPointMatcher::MatchingFilter::UNIDIRECTIONAL;
    } else if (matching_filter == "BIDIRECTIONAL") {
        return StereoKeyPointMatcher::MatchingFilter::BIDIRECTIONAL;
    } else if (matching_filter == "RATIO_TEST") {
        return StereoKeyPointMatcher::MatchingFilter::RATIO_TEST;
    } else {
        throw std::runtime_error("Unrecognised StereoKeyPointMatcher::MatchingFilter \"" + matching_filter + "\"");
    }
}

StereoKeyPointMatcher::StereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold,
        const MatchingFilter matching_filter, const double ratio)
    : window(window),
      vertical_pixel_threshold(vertical_pixel_threshold),
      matching_filter(matching_filter),
      ratio(ratio) {
    if (window.width < 1 || window.width % 2 == 0 || window.height < 1 || window.height % 2 == 0) {
        throw std::runtime_error("Window must have positive odd-length dimensions.");
    }
    half_window_floor = cv::Size((window.width - 1) / 2, (window.height - 1) / 2);
    half_window = cv::Point2f{static_cast<float>(window.width) / 2.f, static_cast<float>(window.height) / 2.f};
    if (ratio <= 0.0 || ratio >= 1.0) {
        throw std::runtime_error("Ratio was " + std::to_string(ratio) + " by must be in range (0, 1).");
    }
}

cv::KeyPoint StereoKeyPointMatcher::keypoint_in_other_image(const cv::KeyPoint& image_keypoint,
        const cv::Mat& left_image, const cv::Mat& right_image, const bool find_in_right, double& cost,
        bool& matched) const {
    // Error checking
    if (left_image.size() != right_image.size()) {
        throw std::runtime_error("Left and right images must be the same size.");
    }

    // Setup
    const cv::Mat& reference_image = find_in_right ? left_image : right_image;
    const cv::Mat& search_image = find_in_right ? right_image : left_image;

    // Compute the top left coordinate
    const cv::Point2i reference_image_top_left = top_left(image_keypoint.pt);

    // Validity check
    if (!window_within_image(reference_image, reference_image_top_left, window)) {
        return cv::KeyPoint{};
    }

    // Generate cost function
    const MatchingCostFunction cost_function = generate_cost_function(reference_image, reference_image_top_left);

    // Iterate over valid x coordinates
    cv::Point2i search_image_top_left = reference_image_top_left;
    int min_cost_top_left_x = search_image_top_left.x;
    cost = std::numeric_limits<double>::max();
    double second_cost{cost};
    const int increment = find_in_right ? -1 : 1;
    while ((find_in_right && search_image_top_left.x >= 0) ||
            (!find_in_right && search_image_top_left.x <= left_image.cols - window.width)) {
        const double window_cost =
                cost_function(reference_image, search_image, reference_image_top_left, search_image_top_left, window);
        if (window_cost < cost) {
            second_cost = cost;
            cost = window_cost;
            min_cost_top_left_x = search_image_top_left.x;
        }
        search_image_top_left.x += increment;
    }
    search_image_top_left.x = min_cost_top_left_x;
    matched = true;
    if (matching_filter == MatchingFilter::BIDIRECTIONAL) {
        // Matching fails if a different window is found with a lower cost
        cv::Point2i reverse_reference_image_top_left = search_image_top_left;
        while ((!find_in_right && reverse_reference_image_top_left.x >= 0) ||
                (find_in_right && reverse_reference_image_top_left.x <= left_image.cols - window.width)) {
            const double window_cost = cost_function(
                    reference_image, search_image, reverse_reference_image_top_left, search_image_top_left, window);
            if (reverse_reference_image_top_left.x != reference_image_top_left.x && window_cost < cost) {
                matched = false;
                break;
            }
            reverse_reference_image_top_left.x -= increment;
        }
    } else if (matching_filter == MatchingFilter::RATIO_TEST && cost / second_cost > ratio) {
        matched = false;
    }

    // Return the keypoint in the middle of the pixel
    cv::KeyPoint search_image_keypoint{image_keypoint};
    search_image_keypoint.pt = cv::Point2f{search_image_top_left} + half_window;
    return search_image_keypoint;
}

std::vector<cv::KeyPoint> StereoKeyPointMatcher::keypoints_in_other_image(
        const std::vector<cv::KeyPoint>& image_keypoints, const cv::Mat& left_image, const cv::Mat& right_image,
        const bool find_in_right, std::vector<double>& costs, std::vector<bool>& matched) const {
    std::vector<cv::KeyPoint> other_image_keypoints(image_keypoints.size());
    costs.resize(image_keypoints.size());
    matched.resize(image_keypoints.size());
    for (std::size_t i = 0; i < other_image_keypoints.size(); ++i) {
        bool match;
        other_image_keypoints[i] =
                keypoint_in_other_image(image_keypoints[i], left_image, right_image, find_in_right, costs[i], match);
        matched[i] = match;
    }
    return other_image_keypoints;
}

int StereoKeyPointMatcher::keypoint_index_in_other_image(const cv::KeyPoint& image_keypoint,
        const std::vector<cv::KeyPoint>& search_image_keypoints, const cv::Mat& left_image, const cv::Mat& right_image,
        const bool find_in_right, double& cost) const {
    // Error checking
    if (left_image.size() != right_image.size()) {
        throw std::runtime_error("Left and right images must be the same size.");
    }

    // Setup
    const cv::Mat& reference_image = find_in_right ? left_image : right_image;
    const cv::Mat& search_image = find_in_right ? right_image : left_image;

    // Compute the top left coordinate
    const cv::Point2i reference_image_top_left = top_left(image_keypoint.pt);

    // Validity check
    if (!window_within_image(reference_image, reference_image_top_left, window)) {
        return -1;
    }

    // Generate cost function
    const MatchingCostFunction cost_function = generate_cost_function(reference_image, reference_image_top_left);

    // Find the min cost other image keypoint
    int min_cost_index{-1};
    cost = std::numeric_limits<double>::max();
    double second_cost{cost};
    for (int i = 0; i < static_cast<int>(search_image_keypoints.size()); ++i) {
        const cv::KeyPoint& search_image_keypoint = search_image_keypoints[i];
        // Check keypoint is within vertical pixel threshold
        if (std::abs(search_image_keypoint.pt.y - image_keypoint.pt.y) <= vertical_pixel_threshold) {
            const cv::Point2i search_image_top_left = top_left(search_image_keypoint.pt);
            if (((find_in_right && search_image_keypoint.pt.x <= image_keypoint.pt.x) ||
                        (!find_in_right && search_image_keypoint.pt.x >= image_keypoint.pt.x)) &&
                    window_within_image(search_image, search_image_top_left, window)) {
                const double window_cost = cost_function(
                        reference_image, search_image, reference_image_top_left, search_image_top_left, window);
                if (window_cost < cost) {
                    second_cost = cost;
                    cost = window_cost;
                    min_cost_index = i;
                }
            }
        }
    }
    if (matching_filter == MatchingFilter::RATIO_TEST && cost / second_cost > ratio) {
        min_cost_index = -1;
    }
    return min_cost_index;
}

std::vector<int> StereoKeyPointMatcher::keypoint_indices_in_other_image(
        const std::vector<cv::KeyPoint>& left_image_keypoints, const std::vector<cv::KeyPoint>& right_image_keypoints,
        const cv::Mat& left_image, const cv::Mat& right_image, const bool find_in_right,
        std::vector<double>& costs) const {
    const std::vector<cv::KeyPoint>& image_keypoints = find_in_right ? left_image_keypoints : right_image_keypoints;
    const std::vector<cv::KeyPoint>& other_image_keypoints =
            find_in_right ? right_image_keypoints : left_image_keypoints;
    std::vector<int> other_image_keypoint_indices(image_keypoints.size());
    costs.resize(other_image_keypoint_indices.size());
    for (std::size_t i = 0; i < other_image_keypoint_indices.size(); ++i) {
        other_image_keypoint_indices[i] = keypoint_index_in_other_image(
                image_keypoints[i], other_image_keypoints, left_image, right_image, find_in_right, costs[i]);
        if (matching_filter == MatchingFilter::BIDIRECTIONAL && other_image_keypoint_indices[i] != -1) {
            double reverse_cost;
            if (keypoint_index_in_other_image(other_image_keypoints[other_image_keypoint_indices[i]], image_keypoints,
                        left_image, right_image, !find_in_right, reverse_cost) != i) {
                other_image_keypoint_indices[i] = -1;
            }
        }
    }
    return other_image_keypoint_indices;
}

cv::Point2i StereoKeyPointMatcher::top_left(const cv::Point2f& centre) const {
    return cv::Point2i{static_cast<int>(std::floor(centre.x)) - half_window_floor.width,
            static_cast<int>(std::floor(centre.y)) - half_window_floor.height};
}

SADStereoKeyPointMatcher::SADStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold,
        const MatchingFilter matching_filter, const double ratio)
    : StereoKeyPointMatcher(window, vertical_pixel_threshold, matching_filter, ratio) {}

cv::Ptr<SADStereoKeyPointMatcher> SADStereoKeyPointMatcher::create(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio) {
    return cv::makePtr<SADStereoKeyPointMatcher>(window, vertical_pixel_threshold, matching_filter, ratio);
}

MatchingCostFunction SADStereoKeyPointMatcher::generate_cost_function(const cv::Mat&, const cv::Point2i&) const {
    return &sum_of_absolute_differences;
}

SSDStereoKeyPointMatcher::SSDStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold,
        const MatchingFilter matching_filter, const double ratio)
    : StereoKeyPointMatcher(window, vertical_pixel_threshold, matching_filter, ratio) {}

cv::Ptr<SSDStereoKeyPointMatcher> SSDStereoKeyPointMatcher::create(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio) {
    return cv::makePtr<SSDStereoKeyPointMatcher>(window, vertical_pixel_threshold, matching_filter, ratio);
}

MatchingCostFunction SSDStereoKeyPointMatcher::generate_cost_function(const cv::Mat&, const cv::Point2i&) const {
    return &sum_of_squared_differences;
}

ZeroMeanSADStereoKeyPointMatcher::ZeroMeanSADStereoKeyPointMatcher(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio)
    : StereoKeyPointMatcher(window, vertical_pixel_threshold, matching_filter, ratio) {}

cv::Ptr<ZeroMeanSADStereoKeyPointMatcher> ZeroMeanSADStereoKeyPointMatcher::create(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio) {
    return cv::makePtr<ZeroMeanSADStereoKeyPointMatcher>(window, vertical_pixel_threshold, matching_filter, ratio);
}

MatchingCostFunction ZeroMeanSADStereoKeyPointMatcher::generate_cost_function(
        const cv::Mat& reference_image, const cv::Point2i& reference_top_left) const {
    // Compute mean of reference image window
    const double reference_window_mean = window_mean(reference_image, reference_top_left, window);

    // Bind the mean to the cost function
    return std::bind(static_cast<double (*)(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&,
                             const cv::Size&, const double)>(&zero_mean_sum_of_absolute_differences),
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5, reference_window_mean);
}

LocallyScaledSADStereoKeyPointMatcher::LocallyScaledSADStereoKeyPointMatcher(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio)
    : StereoKeyPointMatcher(window, vertical_pixel_threshold, matching_filter, ratio) {}

cv::Ptr<LocallyScaledSADStereoKeyPointMatcher> LocallyScaledSADStereoKeyPointMatcher::create(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio) {
    return cv::makePtr<LocallyScaledSADStereoKeyPointMatcher>(window, vertical_pixel_threshold, matching_filter, ratio);
}

MatchingCostFunction LocallyScaledSADStereoKeyPointMatcher::generate_cost_function(
        const cv::Mat& reference_image, const cv::Point2i& reference_top_left) const {
    // Compute mean of reference image window
    const double reference_window_mean = window_mean(reference_image, reference_top_left, window);

    // Bind the mean to the cost function
    return std::bind(static_cast<double (*)(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&,
                             const cv::Size&, const double)>(&locally_scaled_sum_of_absolute_differences),
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5, reference_window_mean);
}

NCCStereoKeyPointMatcher::NCCStereoKeyPointMatcher(const cv::Size& window, const float vertical_pixel_threshold,
        const MatchingFilter matching_filter, const double ratio)
    : StereoKeyPointMatcher(window, vertical_pixel_threshold, matching_filter, ratio) {}

cv::Ptr<NCCStereoKeyPointMatcher> NCCStereoKeyPointMatcher::create(const cv::Size& window,
        const float vertical_pixel_threshold, const MatchingFilter matching_filter, const double ratio) {
    return cv::makePtr<NCCStereoKeyPointMatcher>(window, vertical_pixel_threshold, matching_filter, ratio);
}

MatchingCostFunction NCCStereoKeyPointMatcher::generate_cost_function(
        const cv::Mat& reference_image, const cv::Point2i& reference_top_left) const {
    // Compute mean of reference image window
    const double reference_window_sum_squares = window_sum_squares(reference_image, reference_top_left, window);

    // Bind the mean to the cost function
    return std::bind(static_cast<double (*)(const cv::Mat&, const cv::Mat&, const cv::Point2i&, const cv::Point2i&,
                             const cv::Size&, const double)>(&negative_normalised_cross_correlation),
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5, reference_window_sum_squares);
}
}
