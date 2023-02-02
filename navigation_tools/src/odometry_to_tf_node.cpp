#include <ros/ros.h>

#include "navigation_tools/odometry_to_tf.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_to_tf");
    OdometryToTf odometry_to_tf;
    ros::spin();
    return 0;
}
