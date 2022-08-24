#include <ros/ros.h>

#include "navigation_tools/integrate_imu.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "integrate_imu");
    IntegrateImu integrate_imu;
    ros::spin();
    return 0;
}
