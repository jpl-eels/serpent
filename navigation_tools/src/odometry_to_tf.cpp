#include "navigation_tools/odometry_to_tf.hpp"

#include <geometry_msgs/TransformStamped.h>

OdometryToTf::OdometryToTf()
    : nh("~") {
    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("input", 1, &OdometryToTf::odometry_callback, this);
}

void OdometryToTf::odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry) {
    geometry_msgs::TransformStamped transform;
    transform.header = odometry->header;
    transform.child_frame_id = odometry->child_frame_id;
    transform.transform.translation.x = odometry->pose.pose.position.x;
    transform.transform.translation.y = odometry->pose.pose.position.y;
    transform.transform.translation.z = odometry->pose.pose.position.z;
    transform.transform.translation.z = odometry->pose.pose.position.z;
    transform.transform.rotation = odometry->pose.pose.orientation;
    tf_broadcaster.sendTransform(transform);
}
