#include "camera_tools/split_image.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "split_image");
    camera_tools::ImageSplitter splitter;
    ros::spin();
    return 0;
}
