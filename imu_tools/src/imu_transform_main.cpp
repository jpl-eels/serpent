#include <ros/ros.h>

#include "imu_tools/imu_transform.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_transform");
    ImuTransform imu_transform;
    ros::spin();
    return 0;
}
