#ifndef NAVIGATION_TOOLS_INTEGRATE_IMU_HPP
#define NAVIGATION_TOOLS_INTEGRATE_IMU_HPP

#include <eigen_ros/eigen_ros.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <memory>

// TEST
#include <gtsam/base/types.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class IntegrateImu {
public:
    explicit IntegrateImu();

private:
    void integrate(const sensor_msgs::Imu::ConstPtr& msg);

    //// ROS Communication
    // Node handle
    ros::NodeHandle nh;
    // Odometry publisher
    ros::Publisher odometry_publisher;
    // Path publisher
    ros::Publisher path_publisher;
    // Transform subscriber
    ros::Subscriber imu_subscriber;

    // Extrinsics
    const eigen_ros::BodyFrames body_frames;

    //// IMU Integration
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_params;
    gtsam::imuBias::ConstantBias imu_bias;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> integrator;

    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> integrator_TEST;
    gtsam::NavState state_TEST;
    std::unique_ptr<gtsam::ISAM2> optimiser_TEST;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    gtsam::imuBias::ConstantBias imu_bias_TEST;
    int key{1};

    // State
    ros::Time integration_timestamp;
    gtsam::NavState initial_state;
    nav_msgs::Path path;
};

#endif
