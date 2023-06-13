#ifndef SERPENT_OPTIMISATION_HPP
#define SERPENT_OPTIMISATION_HPP

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include <eigen_ros/body_frames.hpp>
#include <memory>
#include <mutex>

#include "serpent/ImuArray.h"
#include "serpent/StereoFeatures.h"
#include "serpent/back_end/graph_manager.hpp"

namespace serpent {

void from_ros(const std::vector<serpent::StereoFeature>& msgs, std::map<int, gtsam::StereoPoint2>& features);

void to_pcl(const std::map<int, gtsam::Point3>& points, pcl::PointCloud<pcl::PointXYZ>& pointcloud);

void to_pcl(const std::vector<gtsam::Point3>& points, pcl::PointCloud<pcl::PointXYZ>& pointcloud);

gtsam::noiseModel::mEstimator::Base::ReweightScheme to_reweight_scheme(const std::string& reweight_scheme);

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
     * @param features
     */
    void add_stereo_factors(const serpent::StereoFeatures::ConstPtr& features);

    /**
     * @brief Integrate barometer measurement into the factor graph.
     *
     * @param pressure
     */
    void barometer_callback(const sensor_msgs::FluidPressure::ConstPtr& pressure);

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
     * @brief Wrapper function for optimise() and publish().
     *
     * @param key max key for optimisation
     */
    void optimise_and_publish(const int key);

    /**
     * @brief Optimise the graph.
     *
     * @param key max key for optimisation
     * @return gtsam::ISAM2Result
     */
    gtsam::ISAM2Result optimise(const int key);

    /**
     * @brief Publish the results (optimised odometry, IMU bias, global path and global path changes)
     *
     * @param key max key for optimisation
     * @param isam2_result
     */
    void publish(const int key, const gtsam::ISAM2Result& isam2_result);

    /**
     * @brief Perform a number of operations when a crash is detected, potentially including:
     *  - printing information about factors
     *  - saving graph data to file
     *
     * @param ex
     */
    void precrash_operations(const std::exception& ex);

    void print_information_at_key(const gtsam::Key key);

    /**
     * @brief Add transform factor between current and next graph node to graph, optimise, and publish results.
     *
     * Only called if stereo factors disabled and registration factors enabled.
     *
     * @param msg
     */
    void registration_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief Add features and generic stereo factors to graph, optimise, and publish results.
     *
     * Only called if stereo factors enabled and registration factors disabled.
     *
     * @param features
     */
    void stereo_features_callback(const serpent::StereoFeatures::ConstPtr& features);

    /**
     * @brief Update velocity values according to change between consecutive poses (velocity is in the world frame).
     * Useful when velocity isn't part of optimised state, e.g. when IMU isn't optimised.
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
    ros::Publisher stereo_points_publisher;
    ros::Subscriber barometer_subscriber;
    ros::Subscriber imu_s2s_subscriber;
    ros::Subscriber initial_odometry_subscriber;
    ros::Subscriber registration_subscriber;
    ros::Subscriber stereo_features_subscriber;

    //// Thread Management
    mutable std::mutex graph_mutex;

    // Extrinsics
    const eigen_ros::BodyFrames body_frames;
    // Map frame id
    std::string map_frame_id;

    //// Factor Configuration
    bool imu_factors_enabled;
    bool registration_factors_enabled;
    bool stereo_factors_enabled;
    bool barometer_factors_enabled;

    //// IMU Preintegration
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;

    // Graph manager (tracks states, keys, factors, optimisation)
    std::unique_ptr<ISAM2GraphManager> gm;

    //// Barometer
    // Bias noise
    gtsam::SharedNoiseModel barometer_bias_noise;

    //// Debugging
    bool publish_stereo_points;
};

nav_msgs::Path convert_to_path(
        const GraphManager& gm, const int max_key, const std::string& frame_id_prefix, const std::string& map_frame_id);

nav_msgs::Path extract_changed_poses(const nav_msgs::Path& full_path, const gtsam::ISAM2Result& optimisation_result);

}

#endif
