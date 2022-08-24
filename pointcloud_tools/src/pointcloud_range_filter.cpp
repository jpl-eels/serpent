#include "pointcloud_tools/pointcloud_range_filter.hpp"

#include <open3d_conversions/open3d_conversions.h>

#include <limits>

PointcloudRangeFilter::PointcloudRangeFilter()
    : nh("~") {
    publisher = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, &PointcloudRangeFilter::filter_range, this);
    nh.param("min_range", min_range, 0.0);
    nh.param("max_range", max_range, std::numeric_limits<double>::max());
    ROS_INFO_STREAM("Subscribed to " << subscriber.getTopic());
    ROS_INFO_STREAM("Publishing to " << publisher.getTopic());
    ROS_INFO_STREAM("Min Range: " << min_range);
    ROS_INFO_STREAM("Max Range: " << max_range);
}

void PointcloudRangeFilter::filter_range(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Read
    open3d::geometry::PointCloud pointcloud;
    open3d_conversions::rosToOpen3d(msg, pointcloud);

    // Filter
    std::vector<std::size_t> valid_indices;
    for (std::size_t i = 0; i < pointcloud.points_.size(); ++i) {
        const double range{pointcloud.points_[i].norm()};
        if (range >= min_range && range <= max_range) {
            valid_indices.push_back(i);
        }
    }
    std::shared_ptr<open3d::geometry::PointCloud> filtered_pointcloud = pointcloud.SelectByIndex(valid_indices);
    ROS_DEBUG_STREAM(filtered_pointcloud->points_.size() << "/" << pointcloud.points_.size() << " points kept");

    // Write
    sensor_msgs::PointCloud2 output_msg;
    open3d_conversions::open3dToRos(*filtered_pointcloud, output_msg, msg->header.frame_id);
    output_msg.header.stamp = msg->header.stamp;
    publisher.publish(output_msg);
}
