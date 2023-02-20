#ifndef SERPENT_GRAPH_MANAGER_HPP
#define SERPENT_GRAPH_MANAGER_HPP

#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
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
            const gtsam::imuBias::ConstantBias& imu_bias = gtsam::imuBias::ConstantBias(),
            const double barometer_bias = 0.0);
    explicit RobotState(const ros::Time& timestamp, const gtsam::NavState& state,
            const gtsam::imuBias::ConstantBias& imu_bias, const double barometer_bias);

    double barometer_bias() const;

    double& barometer_bias();

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
    gtsam::Pose3 pose_;
    gtsam::Velocity3 velocity_;
    gtsam::imuBias::ConstantBias imu_bias_;
    double barometer_bias_;
};

/**
 * @brief Manager for a pose graph consisting of a chain of robot states.
 *
 */
class GraphManager {
public:
    /**
     * @brief Barometric bias in metres.
     * 
     * @param key 
     * @return double bias in metres
     */
    double barometer_bias(const int key) const;

    /**
     * @brief Barometric bias in metres.
     * 
     * @param key 
     * @param offset 
     * @return double bias in metres
     */
    double barometer_bias(const std::string& key, const int offset) const;

    /**
     * @brief Create a barometric bias (between) factor as a zero-measurement between biases at new_key - 1 and new_key.
     * 
     * @param new_key 
     * @param noise noise model for between factor, where standard devation / sigma is in metres.
     */
    void create_barometric_bias_factor(const int new_key, gtsam::SharedNoiseModel noise);

    /**
     * @brief Create a barometric factor for a new pressure measurement in kilopascals (kPa). Returns the height for
     * that pressure.
     * 
     * @param new_key 
     * @param pressure pressure measurement in kPa
     * @param noise noise model for barometeric factor error, where standard devation / sigma is in metres.
     * @return double height in metres
     */
    double create_barometric_factor(const int new_key, const double pressure, gtsam::SharedNoiseModel noise);

    /**
     * @brief Create a between pose factor between new_key - 1 and new_key. 
     * 
     * @param new_key 
     * @param transform SE(3) relative transform from new_key - 1 to new_key
     * @param noise noise model for between pose factor, with standard devations / sigmas ordered rx, ry, rz, tx, ty, tz
     */
    void create_between_pose_factor(const int new_key, const gtsam::Pose3& transform, gtsam::SharedNoiseModel noise);

    void create_combined_imu_factor(const int new_key, const gtsam::PreintegratedCombinedMeasurements& measurements);

    /**
     * @brief Create a barometric bias prior. This bias is specified in metres.
     *
     * @param key
     * @param barometer_bias vertical distance in metres
     * @param noise
     */
    void create_prior_barometer_bias_factor(const int key, const double barometer_bias, gtsam::SharedNoiseModel noise);

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
     * @brief Get the key value for a named key.
     *
     * @param name
     * @return int
     */
    int key(const std::string& name, const int offset = 0) const;

    /**
     * @brief Return the smallest key value of the named keys with priority <= priority arg (highest = 0).
     *
     * @return int maximum priority of named keys used in the evaluation of the minimum key.
     */
    int minimum_key(const unsigned int priority = std::numeric_limits<unsigned int>::max()) const;

    gtsam::NavState navstate(const int key) const;

    gtsam::NavState navstate(const std::string name, const int offset = 0) const;

    gtsam::Pose3 pose(const int key) const;

    gtsam::Pose3 pose(const std::string name, const int offset = 0) const;

    void print_errors(const double min_error) const;

    unsigned int priority(const std::string key) const;

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

    /**
     * @brief Set the barometer bias in metres.
     *
     * @param key
     * @param barometer_bias
     */
    void set_barometer_bias(const int key, const double barometer_bias);

    /**
     * @brief Set the barometer bias in metres.
     *
     * @param key
     * @param bias
     */
    void set_barometer_bias(const std::string& key, const double barometer_bias, const int offset = 0);

    void set_body_to_stereo_left_cam_pose(const gtsam::Pose3& body_to_stereo_left_cam);

    void set_imu_bias(const int key, const gtsam::imuBias::ConstantBias& imu_bias);

    void set_imu_bias(const std::string& key, const gtsam::imuBias::ConstantBias& imu_bias, const int offset = 0);

    /**
     * @brief Set/create a named key.
     *
     * @param name
     * @param value
     */
    void set_named_key(const std::string& name, const int key = 0, const unsigned int priority = 0);

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

    const ros::Duration time_between(const std::string& key1, const std::string& key2, const int key1_offset = 0,
            const int key2_offset = 0) const;

    const std::map<int, ros::Time>& timestamps() const;

    /**
     * @brief Get the timestamp associated with a particular key value.
     *
     * @param key
     * @return const ros::Time&
     */
    const ros::Time& timestamp(const int key) const;

    /**
     * @brief Get the timestamp associated with a named key.
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
    /**
     * @brief Structure to hold information about a named key. Named keys are used to keep track of the index for which
     * data should be added to the graph.
     *
     */
    struct NamedKeyInfo {
        NamedKeyInfo(const int key = 0, const unsigned int priority = 0)
            : key(key),
              priority(priority) {}

        // Key/index used to acquire gtsam::Key in graph
        int key;

        // Priority of the named key, used in various operations. 0 is maximum priority.
        unsigned int priority;
    };

    void add_factor(const int key, const boost::shared_ptr<gtsam::NonlinearFactor>& factor);

    gtsam::NonlinearFactorGraph all_factors() const;

    /**
     * @brief Extract (find and remove) from new factors all factors with key <= max_key.
     * 
     * @param max_key 
     * @return gtsam::NonlinearFactorGraph 
     */
    gtsam::NonlinearFactorGraph extract_new_factors(const int max_key);

    gtsam::Values extract_new_values(const int max_key);

    gtsam::NonlinearFactorGraph& factors(const int key);

    gtsam::NonlinearFactorGraph& new_factors(const int key);

    template<typename ValueType>
    void add(const gtsam::Key key, const ValueType& value);

    template<typename ValueType>
    void set(const gtsam::Key key, const ValueType& value);

    template<typename ValueType>
    void update(const gtsam::Key key, const ValueType& value);

    // Named keys
    std::map<std::string, NamedKeyInfo> keys;

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

    /**
     * @brief All factors (key -> graph of factors)
     *
     * Contains:
     * - robot state to robot state factors
     * - prior factors
     * - stereo factors (created at that key, which may include factors connecting a landmark with a previous robot
     *      state)
     */
    std::map<int, gtsam::NonlinearFactorGraph> factors_;

    // All values
    gtsam::Values values_;

    // New factors (key -> graph of factors) which have not yet been used in optimisation
    std::map<int, gtsam::NonlinearFactorGraph> new_factors_;

    // New values (key -> values) which have not yet been used in optimisation
    gtsam::Values new_values_;

    // New stereo landmark ids
    std::map<int, std::vector<int>> new_stereo_landmark_ids;
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

    Eigen::Matrix<double, 1, 1> barometer_bias_variance(const int key) const;

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

}

#include "serpent/impl/graph_manager.hpp"

#endif
