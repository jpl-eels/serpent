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
#include <gtsam/slam/StereoFactor.h>
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
    gtsam::Pose3 pose_;                      // X
    gtsam::Velocity3 velocity_;              // V
    gtsam::imuBias::ConstantBias imu_bias_;  // B
};

/**
 * @brief Manager for a pose graph consisting of a chain of robot states.
 *
 */
class GraphManager {
public:
    void create_combined_imu_factor(const int new_key, const gtsam::PreintegratedCombinedMeasurements& measurements);

    void create_between_pose_factor(const int new_key, const gtsam::Pose3& transform, gtsam::SharedNoiseModel noise);

    void create_prior_imu_bias_factor(const int key, const gtsam::imuBias::ConstantBias& imu_bias,
            gtsam::SharedNoiseModel noise);

    void create_prior_pose_factor(const int key, const gtsam::Pose3& pose, gtsam::SharedNoiseModel noise);

    void create_prior_velocity_factor(const int key, const gtsam::Velocity3& velocity, gtsam::SharedNoiseModel noise);

    /**
     * @brief Create the stereo factors and values for a specified key. Requires that:
     *  - the pose has been set for the specified key
     *  - stereo features are not reacquired with the same id if lost
     *
     * Does not require that every consecutive key hold stereo features, i.e. handles missed keys.
     *
     * Throws std::runtime_error otherwise.
     *
     * @param key
     * @param features
     */
    void create_stereo_factors_and_values(const int key, const std::map<int, gtsam::StereoPoint2>& features);

    /**
     * @brief Get all factors between two key bounds inclusive.
     *
     * The key of a unary factors (e.g. priors) is equal to the key of the state.
     *
     * The key of a binary factors (e.g. between factors) is equal to the key of the second state.
     *
     * The key of landmark factors is the key at which they were added.
     *
     * @param first
     * @param last
     * @return gtsam::NonlinearFactorGraph
     */
    gtsam::NonlinearFactorGraph factors(const int first, const int last) const;

    /**
     * @brief Potentially slow function to search through all factors and find those that contain the gtsam key.
     *
     * This is a debugging function, see factors() for standard use.
     *
     * @param key
     * @return gtsam::NonlinearFactorGraph
     */
    gtsam::NonlinearFactorGraph factors_for_key(const gtsam::Key key);

    /**
     * @brief Returns true if the pose has been set for the specified key.
     *
     * @param key
     * @return true
     * @return false
     */
    bool has_pose(const int key) const;

    bool has_pose(const std::string& name, const int offset = 0) const;

    bool has_stereo_landmark(const int id) const;

    gtsam::imuBias::ConstantBias imu_bias(const int key) const;

    gtsam::imuBias::ConstantBias imu_bias(const std::string& name, const int offset = 0) const;

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
    int key(const std::string& name, const int offset = 0) const;

    /**
     * @brief Return the smallest key value of the registered named keys.
     *
     * @return int
     */
    int minimum_key() const;

    gtsam::NavState navstate(const int key) const;

    gtsam::NavState navstate(const std::string name, const int offset = 0) const;

    gtsam::Pose3 pose(const int key) const;

    gtsam::Pose3 pose(const std::string name, const int offset = 0) const;

    void print_errors(const double min_error) const;

    void save(const std::string& file_prefix,
            const gtsam::GraphvizFormatting& formatting = gtsam::GraphvizFormatting{}) const;

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

    void set_body_to_stereo_left_cam_pose(const gtsam::Pose3& body_to_stereo_left_cam);

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

    void set_stereo_calibration(gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration);

    void set_stereo_calibration(const gtsam::Cal3_S2Stereo& stereo_calibration);

    void set_stereo_noise_model(gtsam::SharedNoiseModel noise_model);

    void set_timestamp(const int key, const ros::Time& timestamp);

    void set_timestamp(const std::string& key, const ros::Time& timestamp, const int offset = 0);

    void set_velocity(const int key, const gtsam::Velocity3& velocity);

    void set_velocity(const std::string& key, const gtsam::Velocity3& velocity, const int offset = 0);

    gtsam::Cal3_S2Stereo::shared_ptr stereo_calibration();

    gtsam::StereoCamera stereo_camera(const std::string& key, const int offset = 0) const;

    gtsam::StereoCamera stereo_camera(const int key) const;

    gtsam::Point3 stereo_landmark(const int id) const;

    /**
     * @brief Get the stereo landmarks for a particular key. A key of -1 returns the stereo landmarks for all keys.
     *
     * @param key
     * @return std::map<int, gtsam::Point3>
     */
    std::map<int, gtsam::Point3> stereo_landmarks(const int key = -1) const;

    const ros::Duration time_between(const int key1, const int key2) const;

    const ros::Duration time_between(const std::string& key1_name, const std::string& key2_name,
            const int key1_offset = 0, const int key2_offset = 0) const;

    const std::map<int, ros::Time>& timestamps() const;

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
    const ros::Time& timestamp(const std::string& key, const int offset = 0) const;

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

    /**
     * @brief Return a reference to all values
     *
     * @return const gtsam::Values&
     */
    const gtsam::Values& values() const;

    /**
     * @brief Get a value for a known gtsam key.
     *
     * @param key
     * @return gtsam::Value
     */
    const gtsam::Value& value(const gtsam::Key key) const;

    gtsam::Velocity3 velocity(const int key) const;

    gtsam::Velocity3 velocity(const std::string name, const int offset = 0) const;

protected:
    void add_factor(const int key, const boost::shared_ptr<gtsam::NonlinearFactor>& factor);

    gtsam::NonlinearFactorGraph all_factors() const;

    template<typename ValueType>
    void set(const gtsam::Key key, const ValueType& value);

    // Registered keys
    std::map<std::string, int> keys;

    // All Timestamps (key -> time)
    std::map<int, ros::Time> timestamps_;

    // Stereo Calibration
    gtsam::Cal3_S2Stereo::shared_ptr K;

    // Body to Stereo Left Cam Pose
    boost::optional<gtsam::Pose3> body_to_stereo_left_cam;

    // Stereo Measurement Noise Model
    gtsam::SharedNoiseModel stereo_noise_model;

    // Stereo Features [key = key, value = map[key = id, value = feature]]
    std::map<int, std::map<int, gtsam::StereoPoint2>> stereo_features;

    // Stereo Landmark Ids added to values [key = key when added, value = ids]
    std::map<int, std::vector<int>> stereo_landmark_ids;

    // Values
    gtsam::Values values_;

    /**
     * @brief Factors (key -> graph of factors)
     *
     * Contains:
     * - robot state to robot state factors
     * - prior factors
     * - stereo factors (created at that key, which may include factors connecting a landmark with a previous robot
     *      state)
     */
    std::map<int, gtsam::NonlinearFactorGraph> factors_;
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

    Eigen::MatrixXd covariance(const gtsam::Key key) const;

    Eigen::Matrix<double, 6, 6> imu_bias_covariance(const int key) const;

    Eigen::Matrix<double, 6, 6> pose_covariance(const int key) const;

    Eigen::Matrix3d velocity_covariance(const int key) const;

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
