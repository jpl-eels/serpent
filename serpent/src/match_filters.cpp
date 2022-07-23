#include "serpent/match_filters.hpp"

namespace serpent {

DistanceMatchFilter::DistanceMatchFilter(const float descriptor_distance_threshold):
    descriptor_distance_threshold(descriptor_distance_threshold) {}

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

StereoMatchFilter::StereoMatchFilter(const float vertical_pixel_threshold):
    vertical_pixel_threshold(vertical_pixel_threshold) {}

cv::Ptr<StereoMatchFilter> StereoMatchFilter::create(const float vertical_pixel_threshold) {
    return cv::makePtr<StereoMatchFilter>(vertical_pixel_threshold);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches) {
    return StereoMatchFilter::filter(kp_query, kp_train, matches, vertical_pixel_threshold, indices_);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches,
        const float vertical_pixel_threshold, std::vector<std::size_t>& indices) {
    std::vector<cv::DMatch> filtered_matches;
    indices.clear();
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        const cv::Point2f& query_pt = kp_query.at(match.queryIdx).pt;
        const cv::Point2f& train_pt = kp_train.at(match.trainIdx).pt;
        if (query_pt.x <= train_pt.x && std::abs(query_pt.y - train_pt.y) <= vertical_pixel_threshold) {
            filtered_matches.push_back(match);
            indices.push_back(i);
        }
    }
    return filtered_matches;
}

const std::vector<std::size_t>& StereoMatchFilter::indices() const {
    return indices_;
}

}
