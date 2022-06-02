#include "imu_tools/imu_transform.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_transform");
    ImuTransform imu_transform;
    ros::spin();
    return 0;
}
