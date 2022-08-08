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
    ss << "\t" << "longest tracked match id: " << longest_tracked_match_id << "\n";
    ss << "\t" << "extracted kp #: [" << extracted_kp_count[0] << ", " << extracted_kp_count[1] << "]\n";
    ss << "\t" << "tracked kp #: [" << tracked_kp_count[0] << ", " << tracked_kp_count[1] << "]\n";
    ss << "\t" << "hypothesis match #: " << hypothesis_match_count << "\n";
    ss << "\t" << "hypothesis match id #: " << hypothesis_match_id_count << "\n";
    ss << "\t" << "filtered extracted kp #: [" << filtered_extracted_kp_count[0] << ", "
            << filtered_extracted_kp_count[1] << "]\n";
    ss << "\t" << "tracked match #: " << tracked_match_count << "\n";
    ss << "\t" << "new match #: " << new_match_count << "\n";
    ss << "\t" << "total match #: " << total_match_count << "\n";
    ss << "\t" << "total match id #: " << total_match_id_count << "\n";
    return ss.str();
}

StereoFeatureTracker::StereoFeatureTracker(const cv::Ptr<cv::Feature2D> detector,
        const cv::Ptr<cv::SparseOpticalFlow> sof, const cv::Ptr<StereoMatchFilter> stereo_filter,
        const cv::Ptr<StereoKeyPointMatcher> stereo_matcher, const float new_feature_dist_threshold,
        const double stereo_match_cost_threshold, const cv::Rect2i& roi):
    detector(detector), sof(sof), stereo_filter(stereo_filter), stereo_matcher(stereo_matcher),
    new_feature_sqr_dist_threshold(std::pow(new_feature_dist_threshold, 2.0)),
    stereo_match_cost_threshold(stereo_match_cost_threshold), roi(roi), frame_number(0), next_match_id(0) {}

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
        ROS_DEBUG_STREAM("Created ROI mask");
    }

    // Group images
    LRImages images{{left_image, right_image}};

    // Extract features in left image
    LRKeyPoints new_keypoints;
    for (std::size_t lr = 0; lr < 2; ++lr) {
        new_keypoints[lr] = extract_keypoints(images[lr], roi_mask);
        ROS_DEBUG_STREAM("Extracted keypoints for " << (lr == 0 ? "left" : "right") << " image");
        if (stats) {
            stats->get().extracted_kp_count[lr] = new_keypoints[lr].size();
        }
        if (intermediate_images) {
            cv::drawKeypoints(images[lr], new_keypoints[lr], intermediate_images->get().extracted_keypoints[lr],
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
                cv::drawMatches(previous_images[lr], previous_track_data.keypoints[lr], images[lr],
                        all_sof_keypoints[lr], f2f_matches[lr],
                        intermediate_images->get().sof_matches[lr], intermediate_images->get().new_match_colour,
                        intermediate_images->get().negative_match_colour,
                        std::vector<char>(), intermediate_images->get().match_draw_flags);
            }
        }

        // Remove keypoints too close to those already being tracked
        for (std::size_t lr = 0; lr < 2; ++lr) {
            new_keypoints[lr] = remove_close_keypoints(new_keypoints[lr], new_track_hypotheses.keypoints[lr]);
            ROS_DEBUG_STREAM("Removed keypoints close to already tracked keypoints in the "
                    << (lr == 0 ? "left" : "right") << " frame");
            if (stats) {
                stats->get().filtered_extracted_kp_count[lr] = new_keypoints[lr].size();
            }
        }
    }

    // Filter tracked matches by stereo geometry
    previous_track_data.matches = stereo_filter->filter(new_track_hypotheses.keypoints[0],
            new_track_hypotheses.keypoints[1], new_track_hypotheses.matches);
    previous_track_data.match_ids = extract_match_ids(new_track_hypotheses.match_ids, stereo_filter->indices());
    ROS_DEBUG_STREAM("Filtered matches by stereo geometry");
    if (stats) {
        stats->get().tracked_match_count = previous_track_data.matches.size();
        if (previous_track_data.match_ids.size() > 0) {
            stats->get().longest_tracked_match_id = previous_track_data.match_ids[0];
        } else {
            stats->get().longest_tracked_match_id = -1;
        }
    }
    if (intermediate_images) {
        cv::drawMatches(images[0], new_track_hypotheses.keypoints[0], images[1], new_track_hypotheses.keypoints[1],
                previous_track_data.matches, intermediate_images->get().stereo_filtered_matches,
                intermediate_images->get().tracked_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
    }

    // Extract the keypoints for the matches
    previous_track_data.keypoints = extract_matched_keypoints(new_track_hypotheses.keypoints,
            previous_track_data.matches);
    if (intermediate_images) {
        // Draw the tracked matches
        cv::drawMatches(images[0], previous_track_data.keypoints[0], images[1], previous_track_data.keypoints[1],
                previous_track_data.matches, intermediate_images->get().tracked_matches,
                intermediate_images->get().tracked_match_colour, intermediate_images->get().negative_match_colour,
                std::vector<char>(), intermediate_images->get().match_draw_flags);
    }

    // Find the right image keypoints for the filtered left image detected keypoints using stereo matching
    std::vector<double> stereo_match_costs;
    const std::vector<int> right_keypoint_indices = stereo_matcher->keypoint_indices_in_right_image(new_keypoints[0],
            images[0], images[1], new_keypoints[1], stereo_match_costs);
    ROS_DEBUG_STREAM("Extracted keypoints in right image using stereo matcher");

    // Create the new stereo matches, filtering out high cost and invalid matches
    const LRKeyPointMatches new_filtered_keypoint_matches = create_filtered_new_matches(new_keypoints,
            right_keypoint_indices, stereo_match_costs);
    ROS_DEBUG_STREAM("Filtered new keypoint matches by cost.");
    for (std::size_t i = 0; i < new_filtered_keypoint_matches.size(); ++i) {
        ROS_DEBUG_STREAM("New filtered match: ("
                << new_filtered_keypoint_matches.keypoints[0][new_filtered_keypoint_matches.matches[i].queryIdx].pt.x
                << ", "
                << new_filtered_keypoint_matches.keypoints[0][new_filtered_keypoint_matches.matches[i].trainIdx].pt.y
                << ") <=> ("
                << new_filtered_keypoint_matches.keypoints[1][new_filtered_keypoint_matches.matches[i].queryIdx].pt.x
                << ", "
                << new_filtered_keypoint_matches.keypoints[1][new_filtered_keypoint_matches.matches[i].trainIdx].pt.y
                << "), cost = " << new_filtered_keypoint_matches.matches[i].distance << ", id = "
                << new_filtered_keypoint_matches.match_ids[i]);
    }
    if (stats) {
        stats->get().new_match_count = new_filtered_keypoint_matches.size();
    }
    if (intermediate_images) {
        // Draw the new matches
        cv::drawMatches(images[0], new_filtered_keypoint_matches.keypoints[0], images[1],
                new_filtered_keypoint_matches.keypoints[1], new_filtered_keypoint_matches.matches,
                intermediate_images->get().new_matches, intermediate_images->get().new_match_colour,
                intermediate_images->get().negative_match_colour, std::vector<char>(),
                intermediate_images->get().match_draw_flags);
    }

    // Merge new keypoint matches into track data
    append_keypoint_matches(new_filtered_keypoint_matches, previous_track_data);
    ROS_DEBUG_STREAM("Merged new stereo keypoints into track data");
    if (stats) {
        stats->get().max_match_id = next_match_id - 1;
        stats->get().total_match_count = previous_track_data.matches.size();
        stats->get().total_match_id_count = previous_track_data.match_ids.size();
    }

    // Increment frame number
    ++frame_number;

    // Update the images
    previous_images = images;

    // Return results
    return previous_track_data;
}

void StereoFeatureTracker::append_keypoint_matches(const LRKeyPointMatches& new_keypoint_matches,
        LRKeyPointMatches& track_data) {
    // Append the matches, correcting the match indices
    for (std::size_t i = 0; i < new_keypoint_matches.size(); ++i) {
        track_data.matches.emplace_back(track_data.keypoints[0].size(), track_data.keypoints[1].size(),
                new_keypoint_matches.matches[i].distance);
        track_data.match_ids.emplace_back(new_keypoint_matches.match_ids[i]);
        track_data.keypoints[0].push_back(new_keypoint_matches.keypoints[0][i]);
        track_data.keypoints[1].push_back(new_keypoint_matches.keypoints[1][i]);
    }
}

StereoFeatureTracker::LRKeyPointMatches StereoFeatureTracker::create_filtered_new_matches(
        const LRKeyPoints& new_keypoints, const std::vector<int>& right_indices,
        const std::vector<double>& stereo_match_costs) {
    LRKeyPointMatches new_filtered_keypoint_matches;
    for (std::size_t i = 0; i < new_keypoints[0].size(); ++i) {
        if (stereo_match_costs[i] < stereo_match_cost_threshold && right_indices[i] >= 0) {
            new_filtered_keypoint_matches.matches.emplace_back(new_filtered_keypoint_matches.keypoints[0].size(),
                    new_filtered_keypoint_matches.keypoints[1].size(), stereo_match_costs[i]);
            new_filtered_keypoint_matches.match_ids.emplace_back(next_match_id++);
            new_filtered_keypoint_matches.keypoints[0].push_back(new_keypoints[0][i]);
            new_filtered_keypoint_matches.keypoints[1].push_back(new_keypoints[1][right_indices[i]]);
        }
    }
    return new_filtered_keypoint_matches;
}

std::vector<cv::KeyPoint> StereoFeatureTracker::extract_keypoints(const cv::Mat& image, cv::InputArray roi_) const {
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(image, keypoints, roi_);
    return keypoints;
}

StereoFeatureTracker::LRKeyPoints StereoFeatureTracker::extract_matched_keypoints(const LRKeyPoints& keypoints,
        LRMatches& matches) const {
    LRKeyPoints matched_keypoints;
    for (auto& match : matches) {
        matched_keypoints[0].push_back(keypoints[0][match.queryIdx]);
        matched_keypoints[1].push_back(keypoints[1][match.trainIdx]);
        match.queryIdx = static_cast<int>(matched_keypoints[0].size()) - 1;
        match.trainIdx = static_cast<int>(matched_keypoints[1].size()) - 1;
    }
    return matched_keypoints;
}

StereoFeatureTracker::LRMatchIds StereoFeatureTracker::extract_match_ids(const LRMatchIds& match_ids,
        const std::vector<std::size_t>& indices) const {
    LRMatchIds extracted_match_ids;
    for (const std::size_t index : indices) {
        extracted_match_ids.emplace_back(match_ids[index]);
    }
    return extracted_match_ids;
}

std::vector<cv::KeyPoint> StereoFeatureTracker::remove_close_keypoints(const std::vector<cv::KeyPoint>& keypoints,
        const std::vector<cv::KeyPoint>& reference_keypoints) const {
    // Brute force solution
    std::vector<cv::KeyPoint> filtered_keypoints;
    for (const cv::KeyPoint& kp : keypoints) {
        bool approximately_near_flag{false};
        for (const cv::KeyPoint& reference_kp : reference_keypoints) {
            if (approximately_near(reference_kp, kp, new_feature_sqr_dist_threshold)) {
                approximately_near_flag = true;
                break;
            }
        }
        if (!approximately_near_flag) {
            filtered_keypoints.push_back(kp);
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
