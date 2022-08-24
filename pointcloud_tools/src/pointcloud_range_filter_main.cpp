#include <ros/ros.h>

#include "pointcloud_tools/pointcloud_range_filter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_range_filter");
    PointcloudRangeFilter pointcloud_range_filter;
    ros::spin();
    return 0;
}
