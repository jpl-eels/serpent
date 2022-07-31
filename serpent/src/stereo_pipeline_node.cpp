#include "serpent/stereo_factor_finder.hpp"
#include <eigen_ros/body_frames_tf.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent");
    ros::NodeHandle nh("~");

    // Initialise Modules
    eigen_ros::BodyFramesTf body_frames_tf;
    serpent::StereoFactorFinder stereo_factor_finder;

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
