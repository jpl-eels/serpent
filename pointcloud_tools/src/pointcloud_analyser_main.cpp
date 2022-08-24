#include <ros/ros.h>

#include "pointcloud_tools/pointcloud_analyser.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_analyser");
    PointcloudAnalyser pointcloud_analyser;
    ros::spin();
    return 0;
}
