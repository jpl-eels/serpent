#include "pointcloud_tools/pointcloud_analyser.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <statistics_msgs/SummaryStatisticsArray.h>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

PointcloudAnalyser::PointcloudAnalyser()
    : nh("~") {
    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 100, &PointcloudAnalyser::analyse, this);
    statistics_array_publisher = nh.advertise<statistics_msgs::SummaryStatisticsArray>("output_statistics", 1);
}

void PointcloudAnalyser::analyse(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert pointcloud from ROS
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pointcloud);

    // Print information with statistics
    statistics_msgs::SummaryStatisticsArray statistics_array = statistics(*pointcloud);
    ROS_INFO_STREAM(info_string(*pointcloud, statistics_array.statistics));

    // Publish statistics
    statistics_array_publisher.publish(statistics_array);

    // Print normal information
    if (has_field(*pointcloud, "normal_x") && has_field(*pointcloud, "normal_y") &&
            has_field(*pointcloud, "normal_z")) {
        const int unnormalised_normals = check_normals(*pointcloud);
        if (unnormalised_normals > 0) {
            ROS_WARN_STREAM(unnormalised_normals << "/" << size_points(*pointcloud) << " normals are unnormalised");
        } else {
            ROS_INFO_STREAM("All point normals are normalised.");
        }
    }
}

}
