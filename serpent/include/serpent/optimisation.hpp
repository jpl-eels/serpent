#ifndef SERPENT_OPTIMISATION_HPP
#define SERPENT_OPTIMISATION_HPP

#include "serpent/ImuArray.h"
#include "serpent/StereoLandmarks.h"
#include "serpent/graph_manager.hpp"
#include <eigen_ros/body_frames.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <memory>
#include <mutex>

namespace serpent {

class Optimisation {
public:
    explicit Optimisation();

private:
    /**
     * @brief Add registration data to graph manager.
     * 
     * @param msg 
     */
    void add_registration_factor(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief Add stereo data to graph manager.
     * 
     * @param landmarks 
     */
    void add_stereo_factors(const serpent::StereoLandmarks::ConstPtr& landmarks);

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
     * @brief Run optimisation and publish the ROS output (optimised odometry, IMU bias, global path and global path
     * changes)
     * 
     * @param key max key for optimisation
     */
    void optimise_and_publish(const int key);

    /**
     * @brief Add transform factor between current and next graph node to graph, optimise, and publish results.
     * 
     * Only called if stereo factors disabled and registration factors enabled.
     * 
     * @param msg 
     */
    void registration_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief Combines registration_callback and stereo_landmarks_callback together.
     * 
     * Called if both registration and stereo factors enabled.
     * 
     * @param registration 
     * @param landmarks 
     */
    void registration_stereo_landmarks_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& registration,
            const serpent::StereoLandmarks::ConstPtr& landmarks);

    /**
     * @brief Add landmarks and generic stereo factors to graph, optimise, and publish results.
     * 
     * Only called if stereo factors enabled and registration factors disabled.
     * 
     * @param landmarks 
     */
    void stereo_landmarks_callback(const serpent::StereoLandmarks::ConstPtr& landmarks);

    /**
     * @brief Update velocity values according to linear twist approximation between poses. Useful when IMU isn't
     * optimised.
     * 
     * @param named_key 
     */
    void update_velocity_from_transforms(const std::string& named_key);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher imu_biases_publisher;
    ros::Publisher imu_transform_publisher;
    ros::Publisher optimised_odometry_publisher;
    ros::Publisher path_publisher;
    ros::Publisher path_changes_publisher;
    ros::Subscriber imu_s2s_subscriber;
    ros::Subscriber initial_odometry_subscriber;
    ros::Subscriber registration_subscriber;
    ros::Subscriber stereo_landmarks_subscriber;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> registration_filter_subscriber;
    message_filters::Subscriber<serpent::StereoLandmarks> stereo_landmarks_filter_subscriber;
    message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, serpent::StereoLandmarks> opt_sync;

    //// Thread Management
    mutable std::mutex graph_mutex;

    // Extrinsics
    const eigen_ros::BodyFrames body_frames;

    //// Factor Configuration
    bool imu_factors_enabled;
    bool registration_factors_enabled;
    bool stereo_factors_enabled;

    //// IMU Preintegration
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;

    // Graph manager (tracks states, keys, factors, optimisation)
    std::unique_ptr<ISAM2GraphManager> gm;
};

nav_msgs::Path convert_to_path(const GraphManager& gm, const int max_key, const std::string& frame_id_prefix);

nav_msgs::Path extract_changed_poses(const nav_msgs::Path& full_path, const gtsam::ISAM2Result& optimisation_result);

}

#endif
