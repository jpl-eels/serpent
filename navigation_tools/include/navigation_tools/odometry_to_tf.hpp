#ifndef NAVIGATION_TOOLS_ODOMETRY_TO_TF_HPP
#define NAVIGATION_TOOLS_ODOMETRY_TO_TF_HPP

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class OdometryToTf {
public:
    explicit OdometryToTf();

private:
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry);

    ros::NodeHandle nh;
    ros::Subscriber odometry_subscriber;
    tf2_ros::TransformBroadcaster tf_broadcaster;
};

#endif
