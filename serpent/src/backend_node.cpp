#include <ros/ros.h>

#include <eigen_ros/body_frames_tf.hpp>
#include <memory>

#include "serpent/mapping.hpp"
#include "serpent/optimisation.hpp"
#include "serpent/pointcloud_covariance_estimator.hpp"
#include "serpent/registration.hpp"
#include "serpent/stereo_factor_finder.hpp"

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent_backend");
    ros::NodeHandle nh("serpent");

    // Initialise Modules
    eigen_ros::BodyFramesTf body_frames_tf("serpent");
    serpent::Optimisation optimisation;
    std::unique_ptr<serpent::Mapping<pcl::PointNormal>> mapping;
    std::unique_ptr<serpent::Mapping<PointNormalCovariance>> mapping_with_covariance;
    std::unique_ptr<serpent::Registration<pcl::PointNormal>> registration;
    std::unique_ptr<serpent::Registration<PointNormalCovariance>> registration_with_covariance;
    std::unique_ptr<serpent::StereoFactorFinder> stereo_factor_finder;
    if (nh.param<bool>("optimisation/factors/registration", true)) {
        const bool covariance_estimator_enabled = nh.param<bool>("covariance_estimation/enabled", false);
        const bool point_field_method =
                serpent::is_point_field_method(serpent::to_pointcloud_covariance_estimation_method(
                        nh.param<std::string>("covariance_estimation/method", "RANGE")));
        if (covariance_estimator_enabled && point_field_method) {
            ROS_INFO_STREAM("POINT_FIELD covariance enabled. Building modules with PointNormalCovariance.");
            mapping_with_covariance = std::make_unique<serpent::Mapping<PointNormalCovariance>>();
            registration_with_covariance = std::make_unique<serpent::Registration<PointNormalCovariance>>();
        } else {
            mapping = std::make_unique<serpent::Mapping<pcl::PointNormal>>();
            registration = std::make_unique<serpent::Registration<pcl::PointNormal>>();
        }
    }
    if (nh.param<bool>("optimisation/factors/stereo", true)) {
        stereo_factor_finder = std::make_unique<serpent::StereoFactorFinder>();
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
