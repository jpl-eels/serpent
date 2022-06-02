#include "pointcloud_tools/pointcloud_analyser.hpp"
#include "pointcloud_tools/pclpointcloud2_utilities.hpp"
#include <pcl_conversions/pcl_conversions.h>

PointcloudAnalyser::PointcloudAnalyser():
    nh("~")
{
    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 100, &PointcloudAnalyser::analyse, this);
}

void PointcloudAnalyser::analyse(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert pointcloud from ROS
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pointcloud);

    // Print information about each field
    ROS_INFO_STREAM(pct::info_string(*pointcloud));
}
