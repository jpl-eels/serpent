#include "camera_tools/publish_info.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_info");
    camera_tools::PublishInfo publish_info;
    ros::spin();
    return 0;
}
