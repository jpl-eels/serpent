#ifndef SERPENT_OPTIMISATION_HPP
#define SERPENT_OPTIMISATION_HPP

#include "serpent/ImuArray.h"
#include "serpent/graph_manager.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
    // Temp integrator = todo: delete
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> test_preintegrator;

    // Graph manager (tracks states, keys, factors, optimisation)
    std::unique_ptr<ISAM2GraphManager> gm;
};

nav_msgs::Path convert_to_path(const GraphManager& graph_manager, const int max_key);

nav_msgs::Path extract_changed_poses(const nav_msgs::Path& full_path, const gtsam::ISAM2Result& optimisation_result);

}

#endif
