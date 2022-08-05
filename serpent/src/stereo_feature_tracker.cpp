#include "serpent/stereo_feature_tracker.hpp"
#include <ros/ros.h>
#include <sstream>

namespace serpent {

bool approximately_near(const cv::Point2f& p1, const cv::Point2f& p2, const float sqr_dist) {
    cv::Point2f vec = p2 - p1;
    return std::pow(vec.x, 2.0) + std::pow(vec.y, 2.0) <= sqr_dist;
}

bool approximately_near(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const float sqr_dist) {
    return approximately_near(kp1.pt, kp2.pt, sqr_dist);
}

std::string StereoFeatureTracker::Statistics::to_string() const {
    std::stringstream ss;
    ss << "StereoFeatureTracker::Statistics (frame #" << frame_number << "):\n";
    ss << "\t" << "max match id: " << max_match_id << "\n";
    ss << "\t" << "extracted kp #: [" << extracted_kp_count[0] << ", " << extracted_kp_count[1] << "]\n";
    ss << "\t" << "tracked kp #: [" << tracked_kp_count[0] << ", " << tracked_kp_count[1] << "]\n";
    ss << "\t" << "hypothesis match #: " << hypothesis_match_count << "\n";
    ss << "\t" << "hypothesis match id #: " << hypothesis_match_id_count << "\n";
    ss << "\t" << "filtered extracted kp #: [" << filtered_extracted_kp_count[0] << ", "
            << filtered_extracted_kp_count[1] << "]\n";
    ss << "\t" << "merged kp #: [" << merged_kp_count[0] << ", " << merged_kp_count[1] << "]\n";
    ss << "\t" << "match #: " << match_count << "\n";
    ss << "\t" << "distance filtered match #: " << distance_filtered_match_count << "\n";
    ss << "\t" << "stereo filtered match #: " << stereo_filtered_match_count << "\n";
    ss << "\t" << "consistent match #: " << consistent_match_count << "\n";
    ss << "\t" << "consistent match id #: " << consistent_match_id_count << "\n";
    ss << "\t" << "consistent tracked matches #: " << consistent_tracked_matches << "\n";
    ss << "\t" << "consistent new matches #: " << consistent_new_matches << "\n";
    ss << "\t" << "consistent match kp #: [" << consistent_match_kp_count[0] << ", " << consistent_match_kp_count[1]
            << "]\n";
    ss << "\t" << "longest tracked match id: " << longest_tracked_match_id << "\n";
    return ss.str();
}

StereoFeatureTracker::StereoFeatureTracker(const cv::Ptr<cv::Feature2D> detector,
        const cv::Ptr<cv::Feature2D> descriptor, const cv::Ptr<cv::DescriptorMatcher> matcher,
        const cv::Ptr<cv::SparseOpticalFlow> sof, const cv::Ptr<serpent::StereoMatchFilter> stereo_filter,
        const cv::Ptr<serpent::DistanceMatchFilter> distance_filter, const float new_feature_dist_threshold,
        const cv::Rect2i& roi):
    detector(detector), descriptor(descriptor), matcher(matcher), sof(sof), stereo_filter(stereo_filter),
    distance_filter(distance_filter), new_feature_sqr_dist_threshold(std::pow(new_feature_dist_threshold, 2.0)),
    roi(roi), frame_number(0), next_match_id(0) {}

StereoFeatureTracker::LRKeyPointMatches StereoFeatureTracker::process(const cv::Mat& left_image,
        const cv::Mat& right_image,
        std::optional<std::reference_wrapper<Statistics>> stats, 
        std::optional<std::reference_wrapper<IntermediateImages>> intermediate_images) {
    // Error handling
    if (left_image.size != right_image.size) {
        throw std::runtime_error("Left and right images did not have the same size.");
    }

    // Track statistics
    if (stats) {
        stats->get().frame_number = frame_number;
    }

    // Create mask if ROI is defined and image sizes have changed (or first image)
    if (roi != cv::Rect2i{} && roi_mask.size() != left_image.size()) {
        roi_mask = cv::Mat::zeros(left_image.rows, left_image.cols, CV_8U);
        roi_mask(roi) = 1;
        ROS_DEBUG_STREAM("Create ROI mask");
    }

    // Group images
    LRImages images{{left_image, right_image}};

    // Extract features in both images
    auto keypoints = extract_keypoints(images, roi_mask);
    ROS_DEBUG_STREAM("Extracted keypoints");
    if (stats) {
        stats->get().extracted_kp_count[0] = keypoints[0].size();
        stats->get().extracted_kp_count[1] = keypoints[1].size();
    }
    if (intermediate_images) {
        for (std::size_t lr = 0; lr < 2; ++lr) {
            cv::drawKeypoints(images[lr], keypoints[lr], intermediate_images->get().extracted_keypoints[lr],
                    intermediate_images->get().keypoint_colours, intermediate_images->get().keypoint_draw_flags);
            if (roi != cv::Rect2i{}) {
                cv::rectangle(intermediate_images->get().extracted_keypoints[lr], roi,
                        intermediate_images->get().roi_colour, intermediate_images->get().roi_thickness);
            }
        }
    }

    LRKeyPointMatches new_track_hypotheses;
    if (frame_number > 0) {
        // Track features from previous frame, keeping new keypoints and creating hypothetical new matches
        LRKeyPoints all_sof_keypoints;
        LRF2FMatches f2f_matches;
        new_track_hypotheses = track_previous_keypoints(images, all_sof_keypoints, f2f_matches);
        ROS_DEBUG_STREAM("Tracked previous keypoints");
        if (stats) {
            stats->get().tracked_kp_count[0] = new_track_hypotheses.keypoints[0].size();
            stats->get().tracked_kp_count[1] = new_track_hypotheses.keypoints[1].size();
            stats->get().hypothesis_match_count = new_track_hypotheses.matches.size();
            stats->get().hypothesis_match_id_count = new_track_hypotheses.match_ids.size();
        }
        if (intermediate_images) {
            for (std::size_t lr = 0; lr < 2; ++lr) {
                ROS_DEBUG_STREAM("Previous keypoints #: " << previous_track_data.keypoints[lr].size());
                ROS_DEBUG_STREAM("New Keypoints #: " << all_sof_keypoints[lr].size());
                for (const auto& match : f2f_matches[lr]) {
                    ROS_DEBUG_STREAM("f2f_match: [" << match.queryIdx << ", " << match.trainIdx << "]");
                }
                cv::drawMatches(previous_images[lr], previous_track_data.keypoints[lr], images[lr],
                        all_sof_keypoints[lr], f2f_matches[lr],
                        intermediate_images->get().sof_matches[lr], intermediate_images->get().new_match_colour,
                        intermediate_images->get().negative_match_colour,
                        std::vector<char>(), intermediate_images->get().match_draw_flags);
            }
        }

        // Remove keypoints too close to those already being tracked
        keypoints = remove_already_tracked_keypoints(keypoints, new_track_hypotheses.keypoints);
        ROS_DEBUG_STREAM("Removed already tracked keypoints");
        if (stats) {
            stats->get().filtered_extracted_kp_count[0] = keypoints[0].size();
            stats->get().filtered_extracted_kp_count[1] = keypoints[1].size();
        }
    }

    // Add filtered new keypoints to new track keypoints
    append_keypoints(keypoints, new_track_hypotheses.keypoints);
    ROS_DEBUG_STREAM("Appended keypoints");
    if (stats) {
        stats->get().merged_kp_count[0] = new_track_hypotheses.keypoints[0].size();
        stats->get().merged_kp_count[1] = new_track_hypotheses.keypoints[1].size();
    }
    if (intermediate_images) {
        for (std::size_t lr = 0; lr < 2; ++lr) {
            cv::drawKeypoints(images[lr], new_track_hypotheses.keypoints[lr],
                    intermediate_images->get().merged_keypoints[lr], intermediate_images->get().keypoint_colours,
                    intermediate_images->get().keypoint_draw_flags);
        }
    }

    // Match tracked features in both images
    auto matches = match_tracked_keypoints(images, new_track_hypotheses.keypoints);
    ROS_DEBUG_STREAM("Matched tracked keypoints");
    if (stats) {
        stats->get().match_count = matches.size();
    }
    if (intermediate_images) {
        cv::drawMatches(images[0], new_track_hypotheses.keypoints[0], images[1], new_track_hypotheses.keypoints[1],
                matches, intermediate_images->get().raw_matches, intermediate_images->get().new_match_colour,
                intermediate_images->get().negative_match_colour, std::vector<char>(),
                intermediate_images->get().match_draw_flags);
    }

    // Filter matches
    ROS_DEBUG_STREAM("Filtered matches by distance");
    // Filter matches by distance
    if (distance_filter) {
        matches = distance_filter->filter(matches);
    }
    if (stats) {
        stats->get().distance_filtered_match_count = matches.size();
    }
    if (intermediate_images) {
        cv::drawMatches(images[0], new_track_hypotheses.keypoints[0], images[1], new_track_hypotheses.keypoints[1],
                matches, intermediate_images->get().distance_filtered_matches,
                intermediate_images->get().new_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
    }

    // Filter by stereo geometry
    matches = stereo_filter->filter(new_track_hypotheses.keypoints[0], new_track_hypotheses.keypoints[1], matches);
    ROS_DEBUG_STREAM("Filtered matches by stereo geometry");
    if (stats) {
        stats->get().stereo_filtered_match_count = matches.size();
    }
    if (intermediate_images) {
        cv::drawMatches(images[0], new_track_hypotheses.keypoints[0], images[1], new_track_hypotheses.keypoints[1],
                matches, intermediate_images->get().stereo_filtered_matches,
                intermediate_images->get().new_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
    }

    // Find new and consistent tracked features, and save this as new track data
    const std::size_t consistent_tracked_matches = extract_consistent_matches(matches, new_track_hypotheses.match_ids,
            previous_track_data.matches, previous_track_data.match_ids);
    ROS_DEBUG_STREAM("Extracted consistent matches");
    if (stats) {
        stats->get().consistent_match_count = previous_track_data.matches.size();
        stats->get().consistent_match_id_count = previous_track_data.match_ids.size();
        stats->get().consistent_tracked_matches = consistent_tracked_matches;
        stats->get().consistent_new_matches = stats->get().consistent_match_count - consistent_tracked_matches;
    }

    // Extract the keypoints for the consistent matches
    previous_track_data.keypoints = extract_matched_keypoints(new_track_hypotheses.keypoints,
            previous_track_data.matches);
    ROS_DEBUG_STREAM("Extracted matched keypoints");
    if (stats) {
        stats->get().consistent_match_kp_count[0] = previous_track_data.keypoints[0].size();
        stats->get().consistent_match_kp_count[1] = previous_track_data.keypoints[1].size();
        stats->get().max_match_id = next_match_id - 1;
        if (previous_track_data.match_ids.size() > 0) {
            stats->get().longest_tracked_match_id = previous_track_data.match_ids[0];
        } else {
            stats->get().longest_tracked_match_id = -1;
        }
    }
    if (intermediate_images) {
        // Split matches between tracked and matched
        LRMatches tracked_matches, new_matches;
        for (std::size_t i = 0; i < consistent_tracked_matches; ++i) {
            tracked_matches.push_back(previous_track_data.matches[i]);
        }
        for (std::size_t i = consistent_tracked_matches; i < previous_track_data.matches.size(); ++i) {
            new_matches.push_back(previous_track_data.matches[i]);
        }
        // Draw the new matches
        cv::drawMatches(images[0], previous_track_data.keypoints[0], images[1], previous_track_data.keypoints[1],
                new_matches, intermediate_images->get().consistent_new_matches,
                intermediate_images->get().new_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
        // Draw the tracked matches
        cv::drawMatches(images[0], previous_track_data.keypoints[0], images[1], previous_track_data.keypoints[1],
                tracked_matches, intermediate_images->get().consistent_tracked_matches,
                intermediate_images->get().tracked_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
    }

    // Increment frame number
    ++frame_number;

    // Update the images
    previous_images = images;

    // Return results
    return previous_track_data;
}

void StereoFeatureTracker::append_keypoints(const LRKeyPoints& keypoints, LRKeyPoints& tracked_keypoints)
        const {
    for (std::size_t lr = 0; lr < 2; ++lr) {
        for (const auto& kp : keypoints[lr]) {
            tracked_keypoints[lr].push_back(kp);
        }
    }
}

std::size_t StereoFeatureTracker::extract_consistent_matches(const LRMatches& matches,
        const LRMatchIds& match_hypothesis_ids, LRMatches& consistent_matches, LRMatchIds& consistent_match_ids) {
    // Brute force
    consistent_matches.clear();
    consistent_match_ids.clear();
    std::size_t tracked_match_count{0};
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const auto& match = matches[i];
        if (match.queryIdx < match_hypothesis_ids.size()) {
            // Match has same left and right index means it is a tracked match
            if (match.queryIdx == match.trainIdx) {
                consistent_matches.push_back(match);
                consistent_match_ids.push_back(match_hypothesis_ids[match.queryIdx]);
                ++tracked_match_count;
            }
        } else if (match.trainIdx >= match_hypothesis_ids.size()) {
            consistent_matches.push_back(match);
            consistent_match_ids.push_back(next_match_id++);
        }
    }
    return tracked_match_count;
}

StereoFeatureTracker::LRKeyPoints StereoFeatureTracker::extract_keypoints(const LRImages& images, cv::InputArray roi_)
        const {
    LRKeyPoints keypoints;
    for (std::size_t lr = 0; lr < 2; ++lr) {
        detector->detect(images[lr], keypoints[lr], roi_);
    }
    return keypoints;
}

StereoFeatureTracker::LRKeyPoints StereoFeatureTracker::extract_matched_keypoints(const LRKeyPoints& keypoints,
        LRMatches& matches) {
    LRKeyPoints matched_keypoints;
    for (auto& match : matches) {
        matched_keypoints[0].push_back(keypoints[0][match.queryIdx]);
        matched_keypoints[1].push_back(keypoints[1][match.trainIdx]);
        match.queryIdx = static_cast<int>(matched_keypoints[0].size()) - 1;
        match.trainIdx = static_cast<int>(matched_keypoints[1].size()) - 1;
    }
    return matched_keypoints;
}

StereoFeatureTracker::LRMatches StereoFeatureTracker::match_tracked_keypoints(const LRImages& images,
        LRKeyPoints& keypoints) const {
    LRMatches matches;
    if (keypoints[0].size() > 0 && keypoints[1].size() > 0) {
        LRDescriptors descriptors;
        for (std::size_t lr = 0; lr < 2; ++lr) {
            // Feature description
            descriptor->compute(images[lr], keypoints[lr], descriptors[lr]);
        }

        // Feature matching (best match, note knn and radius are also available)
        matcher->match(descriptors[0], descriptors[1], matches);
    }
    return matches;
}

StereoFeatureTracker::LRKeyPoints StereoFeatureTracker::remove_already_tracked_keypoints(const LRKeyPoints& keypoints,
        const LRKeyPoints& tracked_keypoints) const {
    // Brute force solution
    LRKeyPoints filtered_keypoints;
    for (std::size_t lr = 0; lr < keypoints.size(); ++lr) {
        for (const auto& kp : keypoints[lr]) {
            bool approximately_near_flag{false};
            for (const auto& tracked_kp : tracked_keypoints[lr]) {
                if (approximately_near(tracked_kp, kp, new_feature_sqr_dist_threshold)) {
                    approximately_near_flag = true;
                    break;
                }
            }
            if (!approximately_near_flag) {
                filtered_keypoints[lr].push_back(kp);
            }
        }
    }
    return filtered_keypoints;
}

StereoFeatureTracker::LRKeyPointMatches StereoFeatureTracker::track_previous_keypoints(const LRImages& images,
        LRKeyPoints& all_sof_keypoints, LRF2FMatches& f2f_matches) const {
    LRKeyPointMatches new_track_hypotheses;
    std::array<cv::Mat, 2> sof_points_mats;
    std::array<cv::Mat, 2> sof_status_mats;
    for (std::size_t lr = 0; lr < 2; ++lr) {
        if (previous_track_data.keypoints[lr].size() > 0) {
            // Convert keypoints to matrix
            std::vector<cv::Point2f> prev_points;
            cv::KeyPoint::convert(previous_track_data.keypoints[lr], prev_points);
            const cv::Mat prev_points_mat(prev_points);

            // Sparse Optical Flow
            cv::Mat sof_error_mat;
            sof->calc(previous_images[lr], images[lr], prev_points_mat, sof_points_mats[lr], sof_status_mats[lr],
                    sof_error_mat);

            for (std::size_t i = 0; i < sof_status_mats[lr].rows; ++i) {
                if (sof_status_mats[lr].at<unsigned char>(i) == 1) {
                    f2f_matches[lr].emplace_back(i, i, 0.0f);
                }
                const auto& previous_keypoint_i = previous_track_data.keypoints[lr][i];
                all_sof_keypoints[lr].emplace_back(sof_points_mats[lr].at<cv::Point2f>(i), previous_keypoint_i.size,
                        previous_keypoint_i.angle, previous_keypoint_i.response, previous_keypoint_i.octave,
                        previous_keypoint_i.class_id);
            }
        }
    }

    // Create hypothetical new matches if both status fields were valid
    for (std::size_t i = 0; i < previous_track_data.matches.size(); ++i) {
        const auto& previous_match = previous_track_data.matches[i];
        if (sof_status_mats[0].at<unsigned char>(previous_match.queryIdx) == 1 &&
                sof_status_mats[1].at<unsigned char>(previous_match.trainIdx) == 1) {
            // Create new match with correct new indices, distance is copied although not accurate
            cv::DMatch match{static_cast<int>(new_track_hypotheses.keypoints[0].size()),
                    static_cast<int>(new_track_hypotheses.keypoints[1].size()), previous_match.distance};
            new_track_hypotheses.matches.push_back(match);

            // Add points for the valid matches, keeping the keypoint properties that SOF loses
            const cv::KeyPoint& previous_keypoint_l = previous_track_data.keypoints[0][previous_match.queryIdx];
            const cv::KeyPoint& previous_keypoint_r = previous_track_data.keypoints[1][previous_match.trainIdx];
            new_track_hypotheses.keypoints[0].emplace_back(sof_points_mats[0].at<cv::Point2f>(previous_match.queryIdx),
                    previous_keypoint_l.size, previous_keypoint_l.angle, previous_keypoint_l.response,
                    previous_keypoint_l.octave, previous_keypoint_l.class_id);
            new_track_hypotheses.keypoints[1].emplace_back(sof_points_mats[1].at<cv::Point2f>(previous_match.trainIdx),
                    previous_keypoint_r.size, previous_keypoint_r.angle, previous_keypoint_r.response,
                    previous_keypoint_r.octave, previous_keypoint_r.class_id);

            // Keep the match id
            new_track_hypotheses.match_ids.push_back(previous_track_data.match_ids[i]);
        }
    }

    return new_track_hypotheses;
}

}
