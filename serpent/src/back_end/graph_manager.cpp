#include "serpent/back_end/graph_manager.hpp"

#if GTSAM_VERSION_NUMERIC >= 40200
#include <gtsam/navigation/BarometricFactor.h>
#endif
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/StereoFactor.h>

using gtsam::symbol_shorthand::B;  // IMU Bias       (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::P;  // Pressure Bias  (p)
using gtsam::symbol_shorthand::S;  // Stereo Point3  (x,y,z)
using gtsam::symbol_shorthand::V;  // Vel            (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3          (x,y,z,rx,ry,rz)

namespace serpent {

RobotState::RobotState(const ros::Time& timestamp_, const gtsam::Pose3& pose_, const gtsam::Velocity3& velocity_,
        const gtsam::imuBias::ConstantBias& imu_bias_, const double barometer_bias_)
    : timestamp_(timestamp_),
      pose_(pose_),
      velocity_(velocity_),
      imu_bias_(imu_bias_),
      barometer_bias_(barometer_bias_) {}

RobotState::RobotState(const ros::Time& timestamp_, const gtsam::NavState& state,
        const gtsam::imuBias::ConstantBias& imu_bias_, const double barometer_bias_)
    : RobotState(timestamp_, state.pose(), state.velocity(), imu_bias_, barometer_bias_) {}

double RobotState::barometer_bias() const {
    return barometer_bias_;
}

double& RobotState::barometer_bias() {
    return barometer_bias_;
}

const gtsam::imuBias::ConstantBias& RobotState::imu_bias() const {
    return imu_bias_;
}

gtsam::imuBias::ConstantBias& RobotState::imu_bias() {
    return imu_bias_;
}

gtsam::NavState RobotState::navstate() const {
    return gtsam::NavState(pose_, velocity_);
}

const gtsam::Pose3& RobotState::pose() const {
    return pose_;
}

gtsam::Pose3& RobotState::pose() {
    return pose_;
}

const ros::Time& RobotState::timestamp() const {
    return timestamp_;
}

ros::Time& RobotState::timestamp() {
    return timestamp_;
}

const gtsam::Velocity3& RobotState::velocity() const {
    return velocity_;
}

gtsam::Velocity3& RobotState::velocity() {
    return velocity_;
}

double GraphManager::barometer_bias(const int key_) const {
    assert(key_ >= 0);
    return values_.at<double>(P(key_));
}

double GraphManager::barometer_bias(const std::string& key_, const int offset) const {
    return barometer_bias(key(key_, offset));
}

void GraphManager::create_barometric_bias_factor(const int new_key, gtsam::SharedNoiseModel noise) {
    add_factor(new_key, boost::make_shared<gtsam::BetweenFactor<double>>(P(new_key - 1), P(new_key), 0.0, noise));
}

double GraphManager::create_barometric_factor(const int new_key, const double pressure, gtsam::SharedNoiseModel noise) {
#if GTSAM_VERSION_NUMERIC >= 40200
    auto factor = boost::make_shared<gtsam::BarometricFactor>(X(new_key), P(new_key), pressure, noise);
    add_factor(new_key, factor);
    return factor->heightOut(pressure);
#else
    throw std::runtime_error("BarometricFactor only supported in GTSAM >= 4.2.0. Upgrade GTSAM or disable barometer.");
#endif
}

void GraphManager::create_between_pose_factor(
        const int new_key, const gtsam::Pose3& transform, gtsam::SharedNoiseModel noise) {
    add_factor(new_key,
            boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(new_key - 1), X(new_key), transform, noise));
}

void GraphManager::create_combined_imu_factor(
        const int new_key, const gtsam::PreintegratedCombinedMeasurements& measurements) {
    add_factor(new_key, boost::make_shared<gtsam::CombinedImuFactor>(X(new_key - 1), V(new_key - 1), X(new_key),
                                V(new_key), B(new_key - 1), B(new_key), measurements));
}

void GraphManager::create_prior_barometer_bias_factor(
        const int key_, const double barometer_bias_, gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<double>>(P(key_), barometer_bias_, noise));
}

void GraphManager::create_prior_imu_bias_factor(
        const int key_, const gtsam::imuBias::ConstantBias& imu_bias, gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(key_), imu_bias, noise));
}

void GraphManager::create_prior_pose_factor(const int key_, const gtsam::Pose3& pose, gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key_), pose, noise));
}

void GraphManager::create_prior_velocity_factor(
        const int key_, const gtsam::Velocity3& velocity, gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::Velocity3>>(V(key_), velocity, noise));
}

void GraphManager::create_stereo_factors_and_values(
        const int key_, const std::map<int, gtsam::StereoPoint2>& features) {
    // Save the stereo features
    if (!stereo_features.emplace(key_, features).second) {
        throw std::runtime_error(
                "Failed to set stereo features for key " + std::to_string(key_) + ". Possible duplicate key.");
    }

    // Stereo features for frame and previous (use next because key values are ordered but might not differ by 1)
    const auto features_i = stereo_features.rbegin();
    const auto features_im1 = std::next(features_i);

    // Stereo factors
    auto& factors__ = factors(key_);
    auto& new_factors__ = new_factors(key_);

    // Factors can only be created if the previous frame features were set
    if (features_im1 != stereo_features.rend()) {
        // Stereo features for frame before previous
        const auto features_im2 = std::next(features_im1);

        // Error handling
        if (!has_pose(key_)) {
            throw std::runtime_error(
                    "Failed to create stereo factors and values. Pose X(" + std::to_string(key_) + ") wasn't set.");
        }

        // Stereo Camera (pose corresponds to current feature set)
        const gtsam::StereoCamera camera = stereo_camera(key_);

        // Stereo feature ids
        auto ids_emplace_it = stereo_landmark_ids.emplace(key_, std::vector<int>{});
        auto new_ids_emplace_it = new_stereo_landmark_ids.emplace(key_, std::vector<int>{});
        auto& ids = ids_emplace_it.first->second;
        auto& new_ids = new_ids_emplace_it.first->second;
        for (const auto& [id, feature] : features_i->second) {
            if (features_im1->second.find(id) != features_im1->second.end()) {
                // Add feature and previous state factor if feature is tracked for the first time (not in i-2, in i-1)
                if (features_im2 == stereo_features.rend() ||
                        features_im2->second.find(id) == features_im2->second.end()) {
                    // Previous state factor
                    auto stereo_factor = boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                            features_im1->second.at(id), stereo_noise_model, X(key_ - 1), S(id), K,
                            body_to_stereo_left_cam);
                    factors__.push_back(stereo_factor);
                    new_factors__.push_back(stereo_factor);

                    // Save id
                    ids.emplace_back(id);
                    new_ids.emplace_back(id);
                }

                // Compute landmark position in world frame
                const gtsam::Point3 landmark = camera.backproject(feature);
                // Update values
                set<S>(id, landmark);

                // Add current state factor if feature is in i-1
                auto stereo_factor = boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                        feature, stereo_noise_model, X(key_), S(id), K, body_to_stereo_left_cam);
                factors__.push_back(stereo_factor);
                new_factors__.push_back(stereo_factor);
            }
        }
    }
}

gtsam::NonlinearFactorGraph GraphManager::factors(const int first, const int last) const {
    gtsam::NonlinearFactorGraph extracted_factors;
    for (int i = first; i <= last; ++i) {
        // Factors associated with key i
        auto it = factors_.find(i);
        if (it != factors_.end()) {
            extracted_factors.push_back(it->second);
        }
    }
    return extracted_factors;
}

gtsam::NonlinearFactorGraph GraphManager::factors_for_key(const gtsam::Key key_) {
    gtsam::NonlinearFactorGraph factors_for_key_;
    for (const auto& [key__, factors__] : factors_) {
        for (const auto& factor : factors__) {
            if (factor->find(key_) != factor->end()) {
                factors_for_key_.push_back(factor);
            }
        }
    }
    return factors_for_key_;
}

bool GraphManager::has_pose(const int key_) const {
    assert(key_ >= 0);
    return values_.exists(X(key_));
}

bool GraphManager::has_pose(const std::string& key_, const int offset) const {
    return has_pose(key(key_, offset));
}

bool GraphManager::has_stereo_landmark(const int id) const {
    return values_.exists(S(id));
}

gtsam::imuBias::ConstantBias GraphManager::imu_bias(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::imuBias::ConstantBias>(B(key_));
}

gtsam::imuBias::ConstantBias GraphManager::imu_bias(const std::string& key_, const int offset) const {
    return imu_bias(key(key_, offset));
}

void GraphManager::increment(const std::string& key_) {
    ++keys.at(key_).key;
}

int GraphManager::key(const std::string& key_, const int offset) const {
    return keys.at(key_).key + offset;
}

int GraphManager::minimum_key(const unsigned int priority_) const {
    bool minimum_key_found{false};
    int minimum_key_ = std::numeric_limits<int>::max();
    for (const auto& [name, key_info] : keys) {
        if (key_info.priority <= priority_) {
            minimum_key_ = std::min(minimum_key_, key_info.key);
            minimum_key_found = true;
        }
    }
    if (!minimum_key_found) {
        throw std::runtime_error("No named keys with requested priority <= " + std::to_string(priority_) + " exist.");
    }
    return minimum_key_;
}

gtsam::NavState GraphManager::navstate(const int key_) const {
    return gtsam::NavState{pose(key_), velocity(key_)};
}

gtsam::NavState GraphManager::navstate(const std::string key_, const int offset) const {
    return navstate(key(key_, offset));
}

gtsam::Pose3 GraphManager::pose(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::Pose3>(X(key_));
}

gtsam::Pose3 GraphManager::pose(const std::string key_, const int offset) const {
    return pose(key(key_, offset));
}

void GraphManager::print_errors(const double min_error) const {
    const gtsam::NonlinearFactorGraph all_factors_ = all_factors();
    all_factors_.printErrors(values_, "NonlinearFactorGraph", gtsam::DefaultKeyFormatter,
            [min_error](const gtsam::Factor*, double error, size_t) { return error >= min_error; });
}

unsigned int GraphManager::priority(const std::string key_) const {
    return keys.at(key_).priority;
}

void GraphManager::save(const std::string& file_prefix, const gtsam::GraphvizFormatting& formatting) const {
    const gtsam::NonlinearFactorGraph all_factors_ = all_factors();
    // Saving graph to file was only introduced in GTSAM 4.1.1 so we need to set up the file.
#if GTSAM_VERSION_NUMERIC >= 40200
    all_factors_.saveGraph(file_prefix + ".gv", values(), gtsam::DefaultKeyFormatter, formatting);
#else
    std::ofstream of{file_prefix + ".gv"};
    all_factors_.saveGraph(of, values(), formatting);
    of.close();
#endif
}

RobotState GraphManager::state(const int key_) const {
    assert(key_ >= 0);
    return RobotState{timestamp(key_), pose(key_), velocity(key_), imu_bias(key_), barometer_bias(key_)};
}

RobotState GraphManager::state(const std::string& key_, const int offset) const {
    return state(key(key_, offset));
}

void GraphManager::set_body_to_stereo_left_cam_pose(const gtsam::Pose3& body_to_stereo_left_cam_) {
    body_to_stereo_left_cam = body_to_stereo_left_cam_;
}

void GraphManager::set_barometer_bias(const int key_, const double barometer_bias_) {
    set<P>(key_, barometer_bias_);
}

void GraphManager::set_barometer_bias(const std::string& key_, const double barometer_bias_, const int offset) {
    set_barometer_bias(key(key_, offset), barometer_bias_);
}

void GraphManager::set_imu_bias(const int key_, const gtsam::imuBias::ConstantBias& imu_bias_) {
    assert(key_ >= 0);
    set<B>(key_, imu_bias_);
}

void GraphManager::set_imu_bias(
        const std::string& key_, const gtsam::imuBias::ConstantBias& imu_bias_, const int offset) {
    set_imu_bias(key(key_, offset), imu_bias_);
}

void GraphManager::set_named_key(const std::string& name, const int key_, const unsigned int priority_) {
    if (!keys.emplace(name, NamedKeyInfo{key_, priority_}).second) {
        keys.at(name) = NamedKeyInfo{key_, priority_};
    }
}

void GraphManager::set_navstate(const int key_, const gtsam::NavState& navstate) {
    assert(key_ >= 0);
    set<X>(key_, navstate.pose());
    set<V>(key_, navstate.velocity());
}

void GraphManager::set_navstate(const std::string& key_, const gtsam::NavState& navstate, const int offset) {
    set_navstate(key(key_, offset), navstate);
}

void GraphManager::set_pose(const int key_, const gtsam::Pose3& pose) {
    assert(key_ >= 0);
    set<X>(key_, pose);
}

void GraphManager::set_pose(const std::string& key_, const gtsam::Pose3& pose, const int offset) {
    set_pose(key(key_, offset), pose);
}

void GraphManager::set_stereo_calibration(gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration_) {
    K = stereo_calibration_;
}

void GraphManager::set_stereo_calibration(const gtsam::Cal3_S2Stereo& stereo_calibration_) {
    K = boost::make_shared<gtsam::Cal3_S2Stereo>(stereo_calibration_);
}

void GraphManager::set_stereo_noise_model(gtsam::SharedNoiseModel noise_model) {
    stereo_noise_model = noise_model;
}

void GraphManager::set_timestamp(const int key_, const ros::Time& timestamp_) {
    assert(key_ >= 0);
    auto emplace_it = timestamps_.emplace(key_, timestamp_);
    emplace_it.first->second = timestamp_;
}

void GraphManager::set_timestamp(const std::string& key_, const ros::Time& timestamp_, const int offset) {
    set_timestamp(key(key_, offset), timestamp_);
}

void GraphManager::set_velocity(const int key_, const gtsam::Velocity3& velocity) {
    set<V>(key_, velocity);
}

void GraphManager::set_velocity(const std::string& key_, const gtsam::Velocity3& velocity, const int offset) {
    set_velocity(key(key_, offset), velocity);
}

gtsam::Cal3_S2Stereo::shared_ptr GraphManager::stereo_calibration() {
    return K;
}

gtsam::StereoCamera GraphManager::stereo_camera(const std::string& key_, const int offset) const {
    return stereo_camera(key(key_, offset));
}

gtsam::StereoCamera GraphManager::stereo_camera(const int key_) const {
    const gtsam::Pose3 world_to_stereo_left_cam = pose(key_) * body_to_stereo_left_cam.value();
    return gtsam::StereoCamera{world_to_stereo_left_cam, K};
}

gtsam::Point3 GraphManager::stereo_landmark(const int id) const {
    return values_.at<gtsam::Point3>(S(id));
}

std::map<int, gtsam::Point3> GraphManager::stereo_landmarks(const int key_) const {
    std::map<int, gtsam::Point3> stereo_landmarks_;
    if (key_ == -1) {
        for (const auto [key__, ids] : stereo_landmark_ids) {
            for (const int id : ids) {
                stereo_landmarks_.emplace(id, stereo_landmark(id));
            }
        }
    } else {
        auto it = stereo_landmark_ids.find(key_);
        if (it != stereo_landmark_ids.end()) {
            for (const int id : it->second) {
                stereo_landmarks_.emplace(id, stereo_landmark(id));
            }
        }
    }
    return stereo_landmarks_;
}

const ros::Duration GraphManager::time_between(const int key1, const int key2) const {
    return timestamp(key2) - timestamp(key1);
}

const ros::Duration GraphManager::time_between(
        const std::string& key1, const std::string& key2, const int key1_offset, const int key2_offset) const {
    return time_between(key(key1, key1_offset), key(key2, key2_offset));
}

const ros::Time& GraphManager::timestamp(const int key_) const {
    assert(key_ >= 0);
    return timestamps_.at(key_);
}

const ros::Time& GraphManager::timestamp(const std::string& key_, const int offset) const {
    return timestamp(key(key_, offset));
}

const std::map<int, ros::Time>& GraphManager::timestamps() const {
    return timestamps_;
}

void GraphManager::update_from_values(const gtsam::Values& updated_values_) {
    for (const auto& [key_, value_] : updated_values_) {
        update(key_, value_);
    }
}

void add_values(
        gtsam::Values& extracted_values, const gtsam::Values& values_, const gtsam::Key first, const gtsam::Key last) {
    auto it = values_.lower_bound(first);
    while (it->key >= first && it->key <= last) {
        extracted_values.insert(it->key, it->value);
        ++it;
    }
}

gtsam::Values GraphManager::values(const int first, const int last) const {
    gtsam::Values extracted_values;
    add_values(extracted_values, values_, B(first), B(last));
    add_values(extracted_values, values_, X(first), X(last));
    add_values(extracted_values, values_, V(first), V(last));
    add_values(extracted_values, values_, P(first), P(last));
    // Iterate over (ordered) map of stereo landmark ids
    auto it = stereo_landmark_ids.find(first);
    while (it != stereo_landmark_ids.end() && it->first <= last) {
        for (const int id : it->second) {
            extracted_values.insert(S(id), values_.at(S(id)));
        }
        ++it;
    }
    return extracted_values;
}

const gtsam::Values& GraphManager::values() const {
    return values_;
}

const gtsam::Value& GraphManager::value(const gtsam::Key key_) const {
    return values_.at(key_);
}

gtsam::Velocity3 GraphManager::velocity(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::Velocity3>(V(key_));
}

gtsam::Velocity3 GraphManager::velocity(const std::string key_, const int offset) const {
    return velocity(key(key_, offset));
}

void GraphManager::add_factor(const int key_, const boost::shared_ptr<gtsam::NonlinearFactor>& factor_) {
    // A new graph is added if one doesn't exist already.
    factors(key_).push_back(factor_);
    new_factors(key_).push_back(factor_);
}

gtsam::NonlinearFactorGraph GraphManager::all_factors() const {
    gtsam::NonlinearFactorGraph all_factors_;
    for (const auto& [key_, factors__] : factors_) {
        all_factors_.push_back(factors__);
    }
    return all_factors_;
}

gtsam::NonlinearFactorGraph GraphManager::extract_new_factors(const int max_key) {
    gtsam::NonlinearFactorGraph extracted_new_factors;
    // Iterate over (ordered) map of new factors
    auto it = new_factors_.begin();
    while (it != new_factors_.end() && it->first <= max_key) {
        extracted_new_factors.push_back(it->second);
        // Remove the new factors from the map without invalidating the iterator
        auto it_ = it++;
        new_factors_.erase(it_);
    }
    return extracted_new_factors;
}

gtsam::Values GraphManager::extract_new_values(const int max_key) {
    gtsam::Values extracted_new_values;
    add_values(extracted_new_values, new_values_, B(0), B(max_key));
    add_values(extracted_new_values, new_values_, X(0), X(max_key));
    add_values(extracted_new_values, new_values_, V(0), V(max_key));
    add_values(extracted_new_values, new_values_, P(0), P(max_key));
    // Iterate over (ordered) map of new stereo landmark ids
    auto it = new_stereo_landmark_ids.begin();
    while (it != new_stereo_landmark_ids.end() && it->first <= max_key) {
        for (const int id : it->second) {
            extracted_new_values.insert(S(id), new_values_.at(S(id)));
        }
        // Remove the new stereo_landmark ids from the map without invalidating the iterator
        auto it_ = it++;
        new_stereo_landmark_ids.erase(it_);
    }
    // Remove values from new values
    for (const auto& [key_, value_] : extracted_new_values) {
        new_values_.erase(key_);
    }
    return extracted_new_values;
}

gtsam::FastList<gtsam::Key> GraphManager::extract_unmarginalised_keys(const int max_key) {
    gtsam::FastList<gtsam::Key> extracted_keys_;
    auto it = unmarginalised_keys_.begin();
    while (it != unmarginalised_keys_.end() && it->first <= max_key) {
        extracted_keys_.insert(extracted_keys_.end(), it->second.begin(), it->second.end());
        // Remove the unmarginalised keys from the map without invalidating the iterator
        auto it_ = it++;
        unmarginalised_keys_.erase(it_);
    }
    return extracted_keys_;
}

gtsam::NonlinearFactorGraph& GraphManager::factors(const int key_) {
    return factors_.emplace(key_, gtsam::NonlinearFactorGraph{}).first->second;
}

gtsam::NonlinearFactorGraph& GraphManager::new_factors(const int key_) {
    return new_factors_.emplace(key_, gtsam::NonlinearFactorGraph{}).first->second;
}

gtsam::FastList<gtsam::Key>& GraphManager::unmarginalised_keys(const int key_) {
    return unmarginalised_keys_.emplace(key_, gtsam::FastList<gtsam::Key>{}).first->second;
}

ISAM2GraphManager::ISAM2GraphManager(const gtsam::ISAM2Params& isam2_params)
    : optimiser(isam2_params),
      opt_key_(-1),
      marginalisation(false),
      marginalisation_key_(-1),
      marginalisation_window_size(0) {}

Eigen::Matrix<double, 1, 1> ISAM2GraphManager::barometer_bias_variance(const int key_) const {
    return optimiser.marginalCovariance(P(key_));
}

Eigen::MatrixXd ISAM2GraphManager::covariance(const gtsam::Key key_) const {
    return optimiser.marginalCovariance(key_);
}

Eigen::Matrix<double, 6, 6> ISAM2GraphManager::imu_bias_covariance(const int key_) const {
    return optimiser.marginalCovariance(B(key_));
}

int ISAM2GraphManager::marginalisation_key() const {
    return marginalisation_key_;
}

void ISAM2GraphManager::marginalise(const int key_) {
    if (key_ > opt_key_) {
        throw std::runtime_error("Cannot marginalise before optimisation.");
    }
    auto marginalisation_keys = extract_unmarginalised_keys(key_);
    marginalisation_key_ = key_;
    optimiser.marginalizeLeaves(marginalisation_keys);
}

gtsam::ISAM2Result ISAM2GraphManager::optimise(const int max_key) {
    // Update the optimiser with new factors and new values
    const gtsam::ISAM2Result result = optimiser.update(extract_new_factors(max_key), extract_new_values(max_key));
    opt_key_ = max_key;
    update_from_values(optimiser.calculateEstimate());
    const int new_marginalisation_key = opt_key_ - marginalisation_window_size;
    if (marginalisation && marginalisation_key_ <= new_marginalisation_key) {
        marginalise(new_marginalisation_key);
    }
    return result;
}

int ISAM2GraphManager::opt_key() const {
    return opt_key_;
}

Eigen::Matrix<double, 6, 6> ISAM2GraphManager::pose_covariance(const int key_) const {
    return optimiser.marginalCovariance(X(key_));
}

void ISAM2GraphManager::set_marginalisation(const bool enabled, const unsigned int window_size) {
    marginalisation = enabled;
    if (enabled) {
        marginalisation_window_size = window_size;
    }
}

Eigen::Matrix3d ISAM2GraphManager::velocity_covariance(const int key_) const {
    return optimiser.marginalCovariance(V(key_));
}

}
