#include "pointcloud_tools/pointcloud_analyser.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

PointcloudAnalyser::PointcloudAnalyser()
    : nh("~") {
    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 100, &PointcloudAnalyser::analyse, this);
}

void PointcloudAnalyser::analyse(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert pointcloud from ROS
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pointcloud);

    // Print information about each field
    ROS_INFO_STREAM(pct::info_string(*pointcloud));

    // Print normal information
    if (pct::has_field(*pointcloud, "normal_x") && pct::has_field(*pointcloud, "normal_y") &&
            pct::has_field(*pointcloud, "normal_z")) {
        const int unnormalised_normals = pct::check_normals(*pointcloud);
        if (unnormalised_normals > 0) {
            ROS_WARN_STREAM(
                    unnormalised_normals << "/" << pct::size_points(*pointcloud) << " normals are unnormalised");
        } else {
            ROS_INFO_STREAM("All point normals are normalised.");
        }
    }
}
