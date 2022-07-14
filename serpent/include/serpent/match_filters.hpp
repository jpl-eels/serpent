#ifndef SERPENT_MATCH_FILTERS_HPP
#define SERPENT_MATCH_FILTERS_HPP

#include <opencv2/features2d.hpp>

namespace serpent {

class DistanceMatchFilter {
public:
    DistanceMatchFilter(const float descriptor_distance_threshold = std::numeric_limits<float>::max());

    static cv::Ptr<DistanceMatchFilter> create(const float descriptor_distance_threshold =
            std::numeric_limits<float>::max());

    std::vector<cv::DMatch> filter(const std::vector<cv::DMatch>& matches) const;

    static std::vector<cv::DMatch> filter(const std::vector<cv::DMatch>& matches, const float descriptor_distance_threshold);

protected:
    float descriptor_distance_threshold;
};

/**
 * @brief Filter for removing feature matches that contradict stereo camera geometry.
 * 
 */
class StereoMatchFilter{
public:
    StereoMatchFilter(const float vertical_pixels_threshold = 1.0);

    static cv::Ptr<StereoMatchFilter> create(const float vertical_pixels_threshold = 1.0);

    std::vector<cv::DMatch> filter(const std::vector<cv::KeyPoint>& kp_query, const std::vector<cv::KeyPoint>& kp_train,
            const std::vector<cv::DMatch>& matches) const;

    static std::vector<cv::DMatch> filter(const std::vector<cv::KeyPoint>& kp_query,
            const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches,
            const float vertical_pixels_threshold);

protected:
    float vertical_pixel_threshold;
};

}

#endif
