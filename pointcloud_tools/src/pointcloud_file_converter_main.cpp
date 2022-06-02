#include "pointcloud_tools/pointcloud_file_converter.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_file_converter");
    PointcloudFileConverter pointcloud_file_converter;
    ros::spin();
    return 0;
}
