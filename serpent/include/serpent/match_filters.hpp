#ifndef SERPENT_MATCH_FILTERS_HPP
#define SERPENT_MATCH_FILTERS_HPP

#include <opencv2/features2d.hpp>

namespace serpent {

class DistanceMatchFilter {
public:
    DistanceMatchFilter(const float descriptor_distance_threshold = std::numeric_limits<float>::max());

    static cv::Ptr<DistanceMatchFilter> create(const float descriptor_distance_threshold =
            std::numeric_limits<float>::max());

    std::vector<cv::DMatch> filter(const std::vector<cv::DMatch>& matches);

    static std::vector<cv::DMatch> filter(const std::vector<cv::DMatch>& matches,
            const float descriptor_distance_threshold, std::vector<std::size_t>& indices);

    /**
     * @brief Get indices of match indices from last call to filter (the matches that passed).
     * 
     * @return const std::vector<int>& 
     */
    const std::vector<std::size_t>& indices() const;

protected:
    float descriptor_distance_threshold;

    // State of last filter estimate
    std::vector<std::size_t> indices_;
};

/**
 * @brief Filter for removing feature matches that contradict stereo camera geometry. There are two conditions that must
 * pass:
 *  1. The left x coordinate must be less than the right x coordinate (i.e. features in the left frame are more left).
 *  2. The y coordinates must be within a vertical pixel threshold of each other.
 * 
 */
class StereoMatchFilter{
public:
    StereoMatchFilter(const float vertical_pixels_threshold = 1.0);

    static cv::Ptr<StereoMatchFilter> create(const float vertical_pixels_threshold = 1.0);

    std::vector<cv::DMatch> filter(const std::vector<cv::KeyPoint>& kp_query, const std::vector<cv::KeyPoint>& kp_train,
            const std::vector<cv::DMatch>& matches);

    static std::vector<cv::DMatch> filter(const std::vector<cv::KeyPoint>& kp_query,
            const std::vector<cv::KeyPoint>& kp_train, const std::vector<cv::DMatch>& matches,
            const float vertical_pixels_threshold, std::vector<std::size_t>& indices);

    /**
     * @brief Get indices of match indices from last call to filter (the matches that passed).
     * 
     * @return const std::vector<int>& 
     */
    const std::vector<std::size_t>& indices() const;

protected:
    float vertical_pixel_threshold;

    // State of last filter estimate
    std::vector<std::size_t> indices_;
};

}

#endif
