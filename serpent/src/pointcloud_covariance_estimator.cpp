#include "serpent/pointcloud_covariance_estimator.hpp"

#include <pcl_ros/point_cloud.h>

#include "eigen_ros/eigen_ros.hpp"
#include "pointcloud_tools/pclpointcloud2_covariance.hpp"
#include "pointcloud_tools/pclpointcloud2_utilities.hpp"
#include "statistics_msgs/CovarianceFloat32Stamped.h"

namespace serpent {

PointcloudCovarianceEstimationMethod to_pointcloud_covariance_estimation_method(const std::string& method) {
    if (method == "RANGE") {
        return PointcloudCovarianceEstimationMethod::RANGE;
    } else if (method == "RANGE_BIAS") {
        return PointcloudCovarianceEstimationMethod::RANGE_BIAS;
    }
    throw std::runtime_error("PointcloudCovarianceEstimationMethod \"" + method + "\" not recognised.");
}

bool is_point_field_method(const PointcloudCovarianceEstimationMethod method) {
    switch (method) {
        case PointcloudCovarianceEstimationMethod::RANGE:
            return true;
        case PointcloudCovarianceEstimationMethod::RANGE_BIAS:
            return false;
        default:
            throw std::runtime_error("PointcloudCovarianceEstimationMethod not recognised in is_point_field_method.");
    }
}

PointcloudCovarianceEstimator::PointcloudCovarianceEstimator()
    : nh("serpent") {
    // Configuration
    method = to_pointcloud_covariance_estimation_method(nh.param<std::string>("covariance_estimation/method", "RANGE"));
    nh.param<float>("covariance_estimation/range_noise", range_noise, 0.01f);
    nh.param<float>("covariance_estimation/range_bias_noise", range_bias_noise, 0.01f);

    // Check validity
    if (is_point_field_method(method) && nh.param<bool>("voxel_grid_filter/enabled", true)) {
        ROS_ERROR("VoxelGrid downsampling enabled with covariance as a point field in the point cloud. The covariance "
                  "will be invalid.");
    }

    // Input topic
    const bool filter_enabled = nh.param<bool>("voxel_grid_filter/enabled", true) ||
                                nh.param<bool>("body_filter/enabled", false) ||
                                nh.param<bool>("range_filter/enabled", false) ||
                                nh.param<bool>("statistical_outlier_removal/enabled", false) ||
                                nh.param<bool>("random_sample_filter/enabled", false);
    const std::string input_topic = filter_enabled ? "filter/filtered_pointcloud" : "frontend/deskewed_pointcloud";

    // Publisher and subscribers
    void (serpent::PointcloudCovarianceEstimator::*callback)(const pcl::PCLPointCloud2::ConstPtr&);
    switch (method) {
        case PointcloudCovarianceEstimationMethod::RANGE:
            callback = &PointcloudCovarianceEstimator::compute_range_covariance;
            break;
        case PointcloudCovarianceEstimationMethod::RANGE_BIAS:
            callback = &PointcloudCovarianceEstimator::compute_range_bias_covariance;
            break;
        default:
            throw std::runtime_error("Covariance estimation method not handled.");
    }
    pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, callback, this);
    if (is_point_field_method(method)) {
        pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("covariance_estimator/pointcloud", 1);
    } else {
        covariance_publisher =
                nh.advertise<statistics_msgs::CovarianceFloat32Stamped>("covariance_estimator/covariance", 1);
    }
}

void PointcloudCovarianceEstimator::compute_range_bias_covariance(const pcl::PCLPointCloud2::ConstPtr& msg) {
    const ros::WallTime tic = ros::WallTime::now();
    ROS_DEBUG_STREAM("Starting pointcloud covariance estimation.");

    const Eigen::MatrixXf covariance = pct::compute_range_bias_covariance(*msg, range_noise, range_bias_noise);
    covariance_publisher.publish(eigen_ros::to_ros<statistics_msgs::CovarianceFloat32Stamped>(covariance,
            ros::Time(msg->header.stamp), msg->header.frame_id));

    ROS_INFO_STREAM("Pointcloud covariance estimated in " << (ros::WallTime::now() - tic).toSec() << " s.");
}

void PointcloudCovarianceEstimator::compute_range_covariance(const pcl::PCLPointCloud2::ConstPtr& msg) {
    const ros::WallTime tic = ros::WallTime::now();
    ROS_DEBUG_STREAM("Starting pointcloud covariance estimation.");

    pointcloud_publisher.publish(pct::compute_range_covariance(*msg, range_noise));
    
    ROS_INFO_STREAM("Pointcloud covariance estimated in " << (ros::WallTime::now() - tic).toSec() << " s.");
}

}
