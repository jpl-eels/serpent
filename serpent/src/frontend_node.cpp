#include <ros/ros.h>

#include <pointcloud_tools/point_types.hpp>

#include "serpent/frontend.hpp"
#include "serpent/pointcloud_covariance_estimator.hpp"
#include "serpent/pointcloud_filter.hpp"
#include "serpent/pointcloud_formatter.hpp"
#include "serpent/pointcloud_normal_estimation.hpp"

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent_frontend");
    ros::NodeHandle nh("serpent");

    // Initialise Modules
    serpent::Frontend frontend;
    serpent::PointcloudFormatter pointcloud_formatter;
    std::unique_ptr<serpent::PointcloudFilter> pointcloud_filter;
    std::unique_ptr<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>> normal_estimation;
    std::unique_ptr<serpent::PointcloudNormalEstimation<PointCovariance, PointNormalCovariance>>
            normal_estimation_with_covariance;
    std::unique_ptr<serpent::PointcloudCovarianceEstimator> pointcloud_covariance_estimator;
    if (nh.param<bool>("optimisation/factors/registration", true)) {
        pointcloud_filter = std::make_unique<serpent::PointcloudFilter>();
        const bool covariance_estimator_enabled = nh.param<bool>("covariance_estimation/enabled", false);
        const bool point_field_method =
                serpent::is_point_field_method(serpent::to_pointcloud_covariance_estimation_method(
                        nh.param<std::string>("covariance_estimation/method", "RANGE")));
        if (covariance_estimator_enabled) {
            pointcloud_covariance_estimator = std::make_unique<serpent::PointcloudCovarianceEstimator>();
        }
        if (covariance_estimator_enabled && point_field_method) {
            normal_estimation_with_covariance =
                    std::make_unique<serpent::PointcloudNormalEstimation<PointCovariance, PointNormalCovariance>>();
        } else {
            normal_estimation =
                    std::make_unique<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>>();
        }
    }

    // Start the Node
    const int threads = nh.param<int>("threads", 4);
    if (threads < 1) {
        throw std::runtime_error("Thread count must be >= 1, was " + std::to_string(threads));
    }
    ros::AsyncSpinner spinner(threads);
    spinner.start();
    ROS_INFO_STREAM("Spinning with " << std::to_string(threads) << " threads");
    ros::waitForShutdown();
    std::cerr << "Shutdown complete. Exitting." << std::endl;
    return 0;
}
