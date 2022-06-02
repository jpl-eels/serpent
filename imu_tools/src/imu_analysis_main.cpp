#include "imu_tools/imu_analysis.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_analysis");
    ImuAnalysis imu_analysis;
    ros::spin();
    return 0;
}
