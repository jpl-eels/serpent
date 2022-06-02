#ifndef POINTCLOUD_TOOLS_POINTCLOUD_RANGE_FILTER_HPP
#define POINTCLOUD_TOOLS_POINTCLOUD_RANGE_FILTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointcloudRangeFilter {
public:
    PointcloudRangeFilter();

private:
    /**
     * @brief Filters range
     * 
     * @param msg 
     */
    void filter_range(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // ROS objects
    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    // Configuration
    double min_range;
    double max_range;
};

#endif
