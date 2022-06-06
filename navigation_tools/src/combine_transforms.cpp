#include "navigation_tools/combine_transforms.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

CombineTransforms::CombineTransforms():
    nh("~")
{
    // Publishers
    odometry_publisher = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    path_publisher = nh.advertise<nav_msgs::Path>("path", 1);

    // Subscribers
    const std::string transform_type = nh.param<std::string>("transform_type", "TransformStamped");
    const std::string input{"input"};
    const int queue_size{100};
    if (transform_type == "TransformStamped") {
        subscriber = nh.subscribe<geometry_msgs::TransformStamped>(input, queue_size,
                &CombineTransforms::combine_callback<geometry_msgs::TransformStamped>, this);
    } else if (transform_type == "PoseStamped") {
        subscriber = nh.subscribe<geometry_msgs::PoseStamped>(input, queue_size,
                &CombineTransforms::combine_callback<geometry_msgs::PoseStamped>, this);
    } else if (transform_type == "PoseWithCovarianceStamped") {
        subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(input, queue_size,
                &CombineTransforms::combine_callback<geometry_msgs::PoseWithCovarianceStamped>, this);
    }

    // Path
    path.header.frame_id = "map";
}

void CombineTransforms::combine(const eigen_ros::PoseStamped& transform) {
    // Apply incremental transform
    odometry = eigen_ros::apply_transform(odometry, transform);

    // Convert to ROS
    auto odometry_msg = eigen_ros::to_ros<nav_msgs::Odometry>(odometry);
    odometry_publisher.publish(odometry_msg);

    // Add to path and publish
    path.header.stamp = transform.timestamp;
    eigen_ros::PoseStamped pose_stamped{odometry.pose, odometry.timestamp};
    path.poses.emplace_back(eigen_ros::to_ros<geometry_msgs::PoseStamped>(pose_stamped));
    path_publisher.publish(path);
}
