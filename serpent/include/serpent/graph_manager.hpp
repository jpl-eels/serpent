#ifndef SERPENT_GRAPH_MANAGER_HPP
#define SERPENT_GRAPH_MANAGER_HPP

#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <ros/time.h>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace serpent {

class RobotState {
public:
    explicit RobotState(const ros::Time& timestamp = ros::Time(), const gtsam::Pose3& pose = gtsam::Pose3(),
            const gtsam::Velocity3& velocity = gtsam::Velocity3(),
            const gtsam::imuBias::ConstantBias& imu_bias = gtsam::imuBias::ConstantBias());
    explicit RobotState(const ros::Time& timestamp, const gtsam::NavState& state,
            const gtsam::imuBias::ConstantBias& imu_bias);

    const gtsam::imuBias::ConstantBias& imu_bias() const;

    gtsam::imuBias::ConstantBias& imu_bias();

    gtsam::NavState navstate() const;

    const gtsam::Pose3& pose() const;

    gtsam::Pose3& pose();

    const ros::Time& timestamp() const;

    ros::Time& timestamp();

    const gtsam::Velocity3& velocity() const;

    gtsam::Velocity3& velocity();

private:
    ros::Time timestamp_;
    gtsam::Pose3 pose_; // X
    gtsam::Velocity3 velocity_; // V
    gtsam::imuBias::ConstantBias imu_bias_; // B
};

/**
 * @brief Manager for a pose graph consisting of a chain of robot states.
 * 
 */
class GraphManager {
public:
    void create_combined_imu_factor(const int new_key,
            const gtsam::PreintegratedCombinedMeasurements& measurements);

    void create_between_pose_factor(const int new_key, const gtsam::Pose3& transform,
            gtsam::SharedNoiseModel noise);

    void create_prior_imu_bias_factor(const int key, const gtsam::imuBias::ConstantBias& imu_bias,
            gtsam::SharedNoiseModel noise);

    void create_prior_pose_factor(const int key, const gtsam::Pose3& pose, gtsam::SharedNoiseModel noise);

    void create_prior_velocity_factor(const int key, const gtsam::Velocity3& velocity,
            gtsam::SharedNoiseModel noise);

    /**
     * @brief Get all factors between two key bounds inclusive.
     * 
     * The key of a unary factors (e.g. priors) is equal to the key of the state.
     * 
     * The key of a binary factors (e.g. between factors) is equal to the key of the second state.
     * 
     * @param first 
     * @param last 
     * @return gtsam::NonlinearFactorGraph 
     */
    gtsam::NonlinearFactorGraph factors(const int first, const int last) const;

    gtsam::imuBias::ConstantBias imu_bias(const int key) const;
    
    gtsam::imuBias::ConstantBias imu_bias(const std::string name, const int offset = 0) const;

    /**
     * @brief Increment a named key.
     * 
     * @param name
     */
    void increment(const std::string& name);

    /**
     * @brief Get the key value for a registered named key.
     * 
     * @param name 
     * @return int
     */
    int key(const std::string& name) const;

    gtsam::NavState navstate(const int key) const;
    
    gtsam::NavState navstate(const std::string name, const int offset = 0) const;

    gtsam::Pose3 pose(const int key) const;
    
    gtsam::Pose3 pose(const std::string name, const int offset = 0) const;

    /**
     * @brief Convenience function that returns the state associated with a particular key value.
     * 
     * See also GraphManager::last_state.
     * 
     * @param value 
     * @return const RobotState& 
     */
    RobotState state(const int key) const;

    /**
     * @brief Convenience function that returns the state for a particular named key (optionally with an offset).
     * 
     * @param name 
     * @param offset 
     * @return RobotState 
     */
    RobotState state(const std::string& name, const int offset = 0) const;

    void set_imu_bias(const int key, const gtsam::imuBias::ConstantBias& imu_bias);

    void set_imu_bias(const std::string& key, const gtsam::imuBias::ConstantBias& imu_bias, const int offset = 0);

    /**
     * @brief Set/create a named key.
     * 
     * @param name 
     * @param value 
     */
    void set_named_key(const std::string& name, const int value = 0);

    void set_navstate(const int key, const gtsam::NavState& navstate);

    void set_navstate(const std::string& key, const gtsam::NavState& navstate, const int offset = 0);

    void set_pose(const int key, const gtsam::Pose3& pose);

    void set_pose(const std::string& key, const gtsam::Pose3& pose, const int offset = 0);

    void set_timestamp(const int key, const ros::Time& timestamp);

    void set_timestamp(const std::string& key, const ros::Time& timestamp, const int offset = 0);

    void set_velocity(const int key, const gtsam::Velocity3& velocity);

    void set_velocity(const std::string& key, const gtsam::Velocity3& velocity, const int offset = 0);

    const std::vector<ros::Time>& timestamps() const;

    /**
     * @brief Get the timestamp associated with a particular key value.
     * 
     * @param key 
     * @return const ros::Time& 
     */
    const ros::Time& timestamp(const int key) const;

    /**
     * @brief Get the timestamp associated with a registered key.
     * 
     * @param name 
     * @return const ros::Time& 
     */
    const ros::Time& timestamp(const std::string& name) const;

    /**
     * @brief Update values within the GraphManager. Any values present within the state manager not present in
     * values will remain in the state manager and not be affected. Will throw an error if there are any new values.
     * 
     * @param values 
     */
    void update_from_values(const gtsam::Values& values);

    /**
     * @brief Return the values between two key bounds (inclusive).
     * 
     * @param first 
     * @param last 
     * @return gtsam::Values 
     */
    gtsam::Values values(const int first, const int last) const;

    gtsam::Velocity3 velocity(const int key) const;

    gtsam::Velocity3 velocity(const std::string name, const int offset = 0) const;

private:
    void add_factor(const int key, const boost::shared_ptr<gtsam::NonlinearFactor>& factor);

    template<typename ValueType>
    void set(const gtsam::Key key, const ValueType& value);

    // Registered keys
    std::map<std::string, int> keys;
    
    // All Timestamps (index = key)
    std::vector<ros::Time> timestamps_;

    // Values
    gtsam::Values values_;

    // Factors (index = key)
    std::vector<gtsam::NonlinearFactorGraph> factors_;
};

class ISAM2GraphManager : public GraphManager {
public:
    explicit ISAM2GraphManager(const gtsam::ISAM2Params& isam2_params);

    /**
     * @brief Perform an incremental optimisation up to max_key, and internally update all optimised values.
     * 
     * @param max_key 
     * @return gtsam::ISAM2Result 
     */
    gtsam::ISAM2Result optimise(const int max_key);

    /**
     * @brief Get the optimisation key (max_key of last optimisation, or -1 if no optimisation has occurred)
     * 
     * @return int 
     */
    int opt_key();

    Eigen::Matrix<double, 6, 6> imu_bias_covariance(const int key);

    Eigen::Matrix<double, 6, 6> pose_covariance(const int key);

    Eigen::Matrix3d velocity_covariance(const int key);

private:
    // Optimiser
    gtsam::ISAM2 optimiser;

    // Optimisation key (separate to user-specifiable named keys)
    int opt_key_;
};

/* Implementation */

template<typename ValueType>
void GraphManager::set(const gtsam::Key key_, const ValueType& value) {
    assert(key_ >= 0);
    if (values_.exists(key_)) {
        values_.update(key_, value);
    } else {
        values_.insert(key_, value);
    }
}

}

#endif
