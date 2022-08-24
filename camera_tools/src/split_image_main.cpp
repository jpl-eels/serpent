#include <ros/ros.h>

#include "camera_tools/split_image.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "split_image");
    camera_tools::ImageSplitter splitter;
    ros::spin();
    return 0;
}
