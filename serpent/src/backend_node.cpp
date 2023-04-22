#include <ros/ros.h>

#include <eigen_ros/body_frames_tf.hpp>
#include <memory>

#include "serpent/mapping.hpp"
#include "serpent/optimisation.hpp"
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
    std::unique_ptr<serpent::Mapping<PointNormalUnit>> mapping_unit_vectors;
    std::unique_ptr<serpent::Registration<pcl::PointNormal>> registration;
    std::unique_ptr<serpent::Registration<PointNormalUnit>> registration_unit_vectors;
    std::unique_ptr<serpent::StereoFactorFinder> stereo_factor_finder;
    if (nh.param<bool>("optimisation/factors/registration", true)) {
        const bool add_unit_vectors =
                serpent::requires_unit_vectors(nh.param<std::string>("registration_covariance/method", "CENSI"),
                        nh.param<std::string>("registration_covariance/point_covariance/method", "RANGE"));
        if (add_unit_vectors) {
            ROS_INFO_STREAM("Add unit vectors enabled. Building modules with PointNormalUnit type.");
            mapping_unit_vectors = std::make_unique<serpent::Mapping<PointNormalUnit>>();
            registration_unit_vectors = std::make_unique<serpent::Registration<PointNormalUnit>>();
        } else {
            ROS_INFO_STREAM("Add unit vectors not enabled. Building modules with pcl::PointNormal type.");
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
