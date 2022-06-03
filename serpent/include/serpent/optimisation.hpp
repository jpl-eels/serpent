#ifndef SERPENT_OPTIMISATION_HPP
#define SERPENT_OPTIMISATION_HPP

#include "serpent/ImuArray.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <memory>
#include <mutex>

namespace serpent {

/**
 * @brief A struct of key indices is kept to prevent race conditions between callbacks. Used after initialisation of
 * graph with prior.
 */
struct Keys {
    // IMU factor key
    gtsam::Key imu{1};
    // Registration key
    gtsam::Key reg{1};
    // Optimisation key
    gtsam::Key opt{1};
};

bool operator==(const Keys& lhs, const Keys rhs);

bool operator!=(const Keys& lhs, const Keys rhs);

class Optimisation {
public:
    explicit Optimisation();

private:
    /**
     * @brief Integrate IMU measurements between current and next graph node, add preintegrated IMU factor to the graph
     * and publish the transform.
     * 
     * @param msg 
     */
    void imu_s2s_callback(const serpent::ImuArray::ConstPtr& msg);

    /**
     * @brief Set initial priors, and republish as optimised odometry. Also publish an empty imu transform.
     * 
     * @param msg 
     */
    void initial_odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief Add transform factor between current and next graph node to graph, optimise, and publish results.
     * 
     * @param msg 
     */
    void registration_transform_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher imu_biases_publisher;
    ros::Publisher imu_transform_publisher;
    ros::Publisher optimised_odometry_publisher;
    ros::Publisher path_publisher;
    ros::Publisher path_changes_publisher;
    ros::Subscriber imu_s2s_subscriber;
    ros::Subscriber initial_odometry_subscriber;
    ros::Subscriber registration_transform_subscriber;

    //// Thread Management
    mutable std::mutex graph_mutex;

    //// Factor Configuration
    bool add_imu_factors;
    bool add_registration_factors;

    //// IMU Preintegration
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;

    //// GTSAM Optimisation
    // Timestamps indexed by key
    std::vector<ros::Time> timestamps;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    std::unique_ptr<gtsam::ISAM2> optimiser;
    Keys keys;
    gtsam::NavState optimised_state;

    //// Current State
    gtsam::imuBias::ConstantBias imu_bias;
};

nav_msgs::Path convert_to_path(const gtsam::Values& values, const std::vector<ros::Time> timestamps,
        const gtsam::Key max_key);

nav_msgs::Path extract_changed_poses(const nav_msgs::Path& full_path, const gtsam::ISAM2Result& optimisation_result);

/* Implementation ****************************************************************************************************/

inline bool operator==(const Keys& lhs, const Keys rhs) {
    return lhs.imu == rhs.imu && lhs.reg == rhs.reg && lhs.opt == rhs.opt;
}

inline bool operator!=(const Keys& lhs, const Keys rhs) {
    return !(lhs == rhs);
}

}

#endif
