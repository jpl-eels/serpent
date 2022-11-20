#ifndef POINTCLOUD_TOOLS_POINTCLOUD_ANALYSER_HPP
#define POINTCLOUD_TOOLS_POINTCLOUD_ANALYSER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace pct {

class PointcloudAnalyser {
public:
    PointcloudAnalyser();

private:
    void analyse(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // ROS objects
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
};

}

#endif
