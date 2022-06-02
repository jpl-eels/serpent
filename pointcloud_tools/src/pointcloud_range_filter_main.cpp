#include "pointcloud_tools/pointcloud_range_filter.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_range_filter");
    PointcloudRangeFilter pointcloud_range_filter;
    ros::spin();
    return 0;
}
