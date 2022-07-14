#include "serpent/match_filters.hpp"

namespace serpent {

DistanceMatchFilter::DistanceMatchFilter(const float descriptor_distance_threshold):
    descriptor_distance_threshold(descriptor_distance_threshold) {}

cv::Ptr<DistanceMatchFilter> DistanceMatchFilter::create(const float descriptor_distance_threshold) {
    return cv::makePtr<DistanceMatchFilter>(descriptor_distance_threshold);
}

std::vector<cv::DMatch> DistanceMatchFilter::filter(const std::vector<cv::DMatch>& matches) const {
    return DistanceMatchFilter::filter(matches, descriptor_distance_threshold);
}

std::vector<cv::DMatch> DistanceMatchFilter::filter(const std::vector<cv::DMatch>& matches,
        const float descriptor_distance_threshold) {
    std::vector<cv::DMatch> filtered_matches;
    for (const cv::DMatch& match : matches) {
        if (match.distance <= descriptor_distance_threshold) {
            filtered_matches.push_back(match);
        }
    }
    return filtered_matches;
}

StereoMatchFilter::StereoMatchFilter(const float vertical_pixel_threshold):
    vertical_pixel_threshold(vertical_pixel_threshold) {}

cv::Ptr<StereoMatchFilter> StereoMatchFilter::create(const float vertical_pixel_threshold) {
    return cv::makePtr<StereoMatchFilter>(vertical_pixel_threshold);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches) const {
    return StereoMatchFilter::filter(kp_query, kp_train, matches, vertical_pixel_threshold);
}

std::vector<cv::DMatch> StereoMatchFilter::filter(const std::vector<cv::KeyPoint>& kp_query,
        const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches,
        const float vertical_pixel_threshold) {
    std::vector<cv::DMatch> filtered_matches;
    for (const cv::DMatch& match : matches) {
        const cv::Point2f& query_pt = kp_query.at(match.queryIdx).pt;
        const cv::Point2f& train_pt = kp_train.at(match.trainIdx).pt;
        if (std::abs(query_pt.y - train_pt.y) <= vertical_pixel_threshold) {
            filtered_matches.push_back(match);
        }
    }
    return filtered_matches;
}

}
