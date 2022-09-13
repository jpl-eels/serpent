#include "serpent/match_filters.hpp"

namespace serpent {

DistanceMatchFilter::DistanceMatchFilter(const float descriptor_distance_threshold)
    : descriptor_distance_threshold(descriptor_distance_threshold) {}

cv::Ptr<DistanceMatchFilter> DistanceMatchFilter::create(const float descriptor_distance_threshold) {
    return cv::makePtr<DistanceMatchFilter>(descriptor_distance_threshold);
}

std::vector<cv::DMatch> DistanceMatchFilter::filter(const std::vector<cv::DMatch>& matches) {
    return DistanceMatchFilter::filter(matches, descriptor_distance_threshold, indices_);
}

std::vector<cv::DMatch> DistanceMatchFilter::filter(const std::vector<cv::DMatch>& matches,
        const float descriptor_distance_threshold, std::vector<std::size_t>& indices) {
    std::vector<cv::DMatch> filtered_matches;
    indices.clear();
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        if (match.distance <= descriptor_distance_threshold) {
            filtered_matches.push_back(match);
            indices.push_back(i);
        }
    }
    return filtered_matches;
}

const std::vector<std::size_t>& DistanceMatchFilter::indices() const {
    return indices_;
}

StereoMatchFilter::StereoMatchFilter(const float vertical_pixel_threshold_)
    : vertical_pixel_threshold_(vertical_pixel_threshold_) {}

cv::Ptr<StereoMatchFilter> StereoMatchFilter::create(const float vertical_pixel_threshold_) {
    return cv::makePtr<StereoMatchFilter>(vertical_pixel_threshold_);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches) {
    return StereoMatchFilter::filter(kp_query, kp_train, matches, vertical_pixel_threshold_, indices_);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches,
        const float vertical_pixel_threshold_, std::vector<std::size_t>& indices) {
    std::vector<cv::DMatch> filtered_matches;
    indices.clear();
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        const cv::Point2f& query_pt = kp_query.at(match.queryIdx).pt;
        const cv::Point2f& train_pt = kp_train.at(match.trainIdx).pt;
        if (query_pt.x > train_pt.x && std::abs(query_pt.y - train_pt.y) <= vertical_pixel_threshold_) {
            filtered_matches.push_back(match);
            indices.push_back(i);
        }
    }
    return filtered_matches;
}

const std::vector<std::size_t>& StereoMatchFilter::indices() const {
    return indices_;
}

void StereoMatchFilter::set_vertical_pixel_threshold(const float updated_vertical_pixel_threshold) {
    vertical_pixel_threshold_ = updated_vertical_pixel_threshold;
}

float StereoMatchFilter::vertical_pixel_threshold() const {
    return vertical_pixel_threshold_;
}

StereoDistanceFilter::StereoDistanceFilter(const float fx, const float baseline, const float max_distance,
        const float min_distance)
    : fx_b(fx * baseline),
      max_distance(max_distance),
      min_distance(min_distance) {}

cv::Ptr<StereoDistanceFilter> StereoDistanceFilter::create(const float fx, const float baseline,
        const float max_distance, const float min_distance) {
    return cv::makePtr<StereoDistanceFilter>(fx, baseline, max_distance, min_distance);
}

std::vector<cv::DMatch> StereoDistanceFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches) {
    std::vector<cv::DMatch> filtered_matches;
    indices_.clear();
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        const cv::Point2f& query_pt = kp_query.at(match.queryIdx).pt;
        const cv::Point2f& train_pt = kp_train.at(match.trainIdx).pt;
        const float horiz_diff = query_pt.x - train_pt.x;
        if (horiz_diff > 0.f) {
            const float distance = fx_b / horiz_diff;
            if (distance >= min_distance && distance <= max_distance) {
                filtered_matches.push_back(match);
                indices_.push_back(i);
            }
        }
    }
    return filtered_matches;
}

const std::vector<std::size_t>& StereoDistanceFilter::indices() const {
    return indices_;
}

void StereoDistanceFilter::set_baseline(const float baseline_) {
    baseline = baseline_;
    fx_b = fx * baseline;
}

void StereoDistanceFilter::set_fx(const float fx_) {
    fx = fx_;
    fx_b = fx * baseline;
}

}
