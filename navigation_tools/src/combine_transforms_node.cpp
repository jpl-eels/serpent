#include "navigation_tools/combine_transforms.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "combine_transforms");
    CombineTransforms combine_transforms;
    ros::spin();
    return 0;
}
