#include "serpent/graph_manager.hpp"
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

namespace serpent {

RobotState::RobotState(const ros::Time& timestamp_, const gtsam::Pose3& pose_,
        const gtsam::Velocity3& velocity_, const gtsam::imuBias::ConstantBias& imu_bias_):
    timestamp_(timestamp_), pose_(pose_), velocity_(velocity_), imu_bias_(imu_bias_) {}

RobotState::RobotState(const ros::Time& timestamp_, const gtsam::NavState& state,
        const gtsam::imuBias::ConstantBias& imu_bias_):
    RobotState(timestamp_, state.pose(), state.velocity(), imu_bias_) {}

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

void GraphManager::create_combined_imu_factor(const int new_key,
        const gtsam::PreintegratedCombinedMeasurements& measurements) {
    add_factor(new_key, boost::make_shared<gtsam::CombinedImuFactor>(X(new_key - 1), V(new_key - 1), X(new_key),
            V(new_key), B(new_key - 1), B(new_key), measurements));
}

void GraphManager::create_between_pose_factor(const int new_key, const gtsam::Pose3& transform,
        gtsam::SharedNoiseModel noise) {
    add_factor(new_key, boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(new_key - 1), X(new_key), transform,
            noise));
}

void GraphManager::create_prior_imu_bias_factor(const int key_, const gtsam::imuBias::ConstantBias& imu_bias,
        gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(key_), imu_bias, noise));
}

void GraphManager::create_prior_pose_factor(const int key_, const gtsam::Pose3& pose,
        gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key_), pose, noise));
    
}

void GraphManager::create_prior_velocity_factor(const int key_, const gtsam::Velocity3& velocity,
        gtsam::SharedNoiseModel noise) {
    add_factor(key_, boost::make_shared<gtsam::PriorFactor<gtsam::Velocity3>>(V(key_), velocity, noise));
}

gtsam::NonlinearFactorGraph GraphManager::factors(const int first, const int last) const {
    gtsam::NonlinearFactorGraph extracted_factors;
    for (int i = first; i <= last; ++i) {
        extracted_factors.push_back(factors_.at(i));
    }
    return extracted_factors;
}

gtsam::imuBias::ConstantBias GraphManager::imu_bias(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::imuBias::ConstantBias>(B(key_));
}

gtsam::imuBias::ConstantBias GraphManager::imu_bias(const std::string key_, const int offset) const {
    return imu_bias(key(key_) + offset);
}

int GraphManager::key(const std::string& name) const {
    return keys.at(name);
}

void GraphManager::increment(const std::string& name) {
    ++keys.at(name);
}

gtsam::NavState GraphManager::navstate(const int key_) const {
    return gtsam::NavState{pose(key_), velocity(key_)};
}

gtsam::NavState GraphManager::navstate(const std::string key_, const int offset) const {
    return navstate(key(key_) + offset);
}

gtsam::Pose3 GraphManager::pose(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::Pose3>(X(key_));
}

gtsam::Pose3 GraphManager::pose(const std::string key_, const int offset) const {
    return pose(key(key_) + offset);
}

RobotState GraphManager::state(const int key_) const {
    assert(key_ >= 0);
    return RobotState{timestamp(key_), pose(key_), velocity(key_), imu_bias(key_)};
}

RobotState GraphManager::state(const std::string& name, const int offset) const {
    return state(key(name) + offset);
}

void GraphManager::set_imu_bias(const int key_, const gtsam::imuBias::ConstantBias& imu_bias) {
    assert(key_ >= 0);
    set(B(key_), imu_bias);
}

void GraphManager::set_imu_bias(const std::string& name, const gtsam::imuBias::ConstantBias& imu_bias,
        const int offset) {
    set_imu_bias(key(name) + offset, imu_bias);
}

void GraphManager::set_named_key(const std::string& name, const int value_) {
    if (!keys.emplace(name, value_).second) {
        keys.at(name) = value_;
    }
}

void GraphManager::set_navstate(const int key_, const gtsam::NavState& navstate) {
    assert(key_ >= 0);
    set(X(key_), navstate.pose());
    set(V(key_), navstate.velocity());
}

void GraphManager::set_navstate(const std::string& name, const gtsam::NavState& navstate, const int offset) {
    set_navstate(key(name) + offset, navstate);
}

void GraphManager::set_pose(const int key_, const gtsam::Pose3& pose) {
    assert(key_ >= 0);
    set(X(key_), pose);
}

void GraphManager::set_pose(const std::string& name, const gtsam::Pose3& pose, const int offset) {
    set_pose(key(name) + offset, pose);
}

void GraphManager::set_timestamp(const int key_, const ros::Time& timestamp_) {
    assert(key_ >= 0);
    if (key_ == static_cast<int>(timestamps_.size())) {
        timestamps_.push_back(timestamp_);
    } else {
        timestamps_.at(key_) = timestamp_;
    }
}

void GraphManager::set_timestamp(const std::string& name, const ros::Time& timestamp_, const int offset) {
    set_timestamp(key(name) + offset, timestamp_);
}

void GraphManager::set_velocity(const int key_, const gtsam::Velocity3& velocity) {
    set(V(key_), velocity);
}

void GraphManager::set_velocity(const std::string& name, const gtsam::Velocity3& velocity, const int offset) {
    set_velocity(key(name) + offset, velocity);
}

const ros::Time& GraphManager::timestamp(const int key_) const {
    assert(key_ >= 0);
    return timestamps_.at(key_);
}

const ros::Time& GraphManager::timestamp(const std::string& name) const {
    return timestamp(key(name));
}

const std::vector<ros::Time>& GraphManager::timestamps() const {
    return timestamps_;
}

void GraphManager::update_from_values(const gtsam::Values& updated_values_) {
    for (const auto& [key_, value_] : updated_values_) {
        values_.update(key_, value_);
    }
}

void add_values(gtsam::Values& extracted_values, const gtsam::Values& values_, const gtsam::Key first,
        const gtsam::Key last) {
    gtsam::Values::const_iterator it = values_.lower_bound(first);
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
    return extracted_values;
}

gtsam::Velocity3 GraphManager::velocity(const int key_) const {
    assert(key_ >= 0);
    return values_.at<gtsam::Velocity3>(V(key_));
}

gtsam::Velocity3 GraphManager::velocity(const std::string key_, const int offset) const {
    return velocity(key(key_) + offset);
}

void GraphManager::add_factor(const int key_, const boost::shared_ptr<gtsam::NonlinearFactor>& factor_) {
    if (key_ >= static_cast<int>(factors_.size())) {
        factors_.resize(key_ + 1);
    } 
    factors_.at(key_).push_back(factor_);
}

ISAM2GraphManager::ISAM2GraphManager(const gtsam::ISAM2Params& isam2_params):
    optimiser(isam2_params), opt_key_(-1)
{}

gtsam::ISAM2Result ISAM2GraphManager::optimise(const int max_key) {
    // Update the optimiser with new factors and new values
    const int min_key = opt_key_ + 1;
    const gtsam::ISAM2Result result = optimiser.update(factors(min_key, max_key), values(min_key, max_key));
    opt_key_ = max_key;
    update_from_values(optimiser.calculateEstimate());
    return result;
}

int ISAM2GraphManager::opt_key() {
    return opt_key_;
}

Eigen::Matrix<double, 6, 6> ISAM2GraphManager::pose_covariance(const int key_) {
    return optimiser.marginalCovariance(X(key_));
}

Eigen::Matrix3d ISAM2GraphManager::velocity_covariance(const int key_) {
    return optimiser.marginalCovariance(V(key_));
}

}

