#include "serpent/frontend.hpp"
#include "serpent/mapping.hpp"
#include "serpent/optimisation.hpp"
#include "serpent/pointcloud_filter.hpp"
#include "serpent/pointcloud_formatter.hpp"
#include "serpent/pointcloud_normal_estimation.hpp"
#include "serpent/registration.hpp"
#include "serpent/stereo_factor_finder.hpp"
#include <eigen_ros/body_frames_tf.hpp>
#include <ros/ros.h>
#include <memory>

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent");
    ros::NodeHandle nh("~");

    // Initialise Modules
    eigen_ros::BodyFramesTf body_frames_tf;
    serpent::Frontend frontend;
    serpent::Mapping mapping;
    serpent::Optimisation optimisation;
    serpent::PointcloudFilter pointcloud_filter;
    serpent::PointcloudFormatter pointcloud_formatter;
    serpent::PointcloudNormalEstimation pointcloud_normal_estimation;
    serpent::Registration registration;
    std::unique_ptr<serpent::StereoFactorFinder> stereo_factor_finder;
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
