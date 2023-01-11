#include <ros/ros.h>

#include "serpent/frontend.hpp"
#include "serpent/pointcloud_filter.hpp"
#include "serpent/pointcloud_formatter.hpp"
#include "serpent/pointcloud_normal_estimation.hpp"

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent");
    ros::NodeHandle nh("~");

    // Initialise Modules
    serpent::Frontend frontend;
    serpent::PointcloudFilter pointcloud_filter;
    serpent::PointcloudFormatter pointcloud_formatter;
    serpent::PointcloudNormalEstimation pointcloud_normal_estimation;

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
