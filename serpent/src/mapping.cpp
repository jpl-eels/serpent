#include "serpent/mapping.hpp"
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/eigen_ros.hpp>
#include <pcl/common/transforms.h>
#include <cmath>

namespace serpent {

MapFrame::MapFrame(const eigen_ros::PoseStamped& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud):
        pose(pose), pointcloud(pointcloud) {}

Mapping::Mapping():
    nh("~"), correct_map_sync(10)
{
    // Publishers
    local_map_publisher = nh.advertise<pcl::PointCloud<pcl::PointNormal>>("mapping/local_map", 1);

    // Subscribers
    pointcloud_subscriber.subscribe(nh, "normal_estimation/pointcloud", 10);
    path_changes_subscriber.subscribe(nh, "optimisation/path_changes", 10);
    correct_map_sync.connectInput(pointcloud_subscriber, path_changes_subscriber);
    correct_map_sync.registerCallback(boost::bind(&Mapping::map_update_callback, this, _1, _2));

    // Configuration
    nh.param<double>("mapping/distance_threshold", distance_threshold, 1.0);
    nh.param<double>("mapping/rotation_threshold", rotation_threshold, M_PI/6.0);
    const int frame_extraction_number_ = nh.param<int>("mapping/frame_extraction_number", 10);
    if (frame_extraction_number_ < 1) {
        throw std::runtime_error("Frame extraction number " + std::to_string(frame_extraction_number_) + " invalid");
    }
    frame_extraction_number = static_cast<std::size_t>(frame_extraction_number_);
}

std::pair<std::map<ros::Time, MapFrame>::iterator, bool> Mapping::add_to_map(const eigen_ros::PoseStamped& pose_stamped,
        const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pointcloud) {
    return map.emplace(pose_stamped.timestamp, MapFrame{pose_stamped, pointcloud});
}

void Mapping::map_update_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pointcloud_msg,
        const nav_msgs::Path::ConstPtr& path_changes_msg) {
    if (path_changes_msg->poses.empty()) {
        throw std::runtime_error("path.poses was empty. Something went wrong.");
    }

    // Iterate over the path:
    // 1. Correct all pointclouds in the global map with matching timestamps
    // 2. Add the new pointcloud (and retain the pointcloud's pose)
    bool pointcloud_timestamp_found{false};
    const ros::Time pointcloud_timestamp = pcl_conversions::fromPCL(pointcloud_msg->header.stamp);
    eigen_ros::PoseStamped body_pose;
    for (const auto& pose : path_changes_msg->poses) {
        eigen_ros::PoseStamped pose_stamped = eigen_ros::from_ros<eigen_ros::PoseStamped>(pose);
        if (pose_stamped.timestamp == pointcloud_timestamp) {
            // Add new pointcloud (at last path pose) into map
            if (map.empty() || should_add_to_map(pose_stamped.data, last_frame().pose.data)) {
                add_to_map(pose_stamped, pointcloud_msg);
                ROS_INFO_STREAM("Added frame at " << pointcloud_timestamp << " to map (" << map.size()
                        << " total keyframes)");
            }
            body_pose = pose_stamped;
            pointcloud_timestamp_found = true;
        } else {
            // Update pose in the map
            auto it = map.find(pose_stamped.timestamp);
            if (it != map.end()) {
                it->second.pose = pose_stamped;
            }
        }
    }
    // Pointcloud's pose must be in the path
    if (!pointcloud_timestamp_found) {
        throw std::runtime_error("Pointcloud timestamp (" + std::to_string(pointcloud_timestamp.toSec()) +
                ") was not in the path poses. Something went wrong.");
    }

    // Extract map around pose in the pose frame
    auto map_region = extract_past_frames(frame_extraction_number, body_pose);

    // Publish the local map
    local_map_publisher.publish(map_region);
}

pcl::PointCloud<pcl::PointNormal>::Ptr Mapping::extract_past_frames(const std::size_t n,
        const eigen_ros::PoseStamped& body_frame) {
    auto it = map.rbegin();
    while (it->first > body_frame.timestamp) {
        ++it;
    }
    auto pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pointcloud->header.stamp = pcl_conversions::toPCL(body_frame.timestamp);
    pointcloud->header.frame_id = "body_i-1";
    // B_{i-1,s}^{W} = (B_{W}^{i-1,s})^-1
    Eigen::Isometry3d body_frame_inv = to_transform(body_frame.data).inverse();
    for (std::size_t i = 0; i < n; ++i) {
        if (it->second.pose.timestamp > body_frame.timestamp) {
            throw std::runtime_error("Timestamp in the future detected. Something went wrong.");
        }
        // B_{i-1,s}^{j,s} = B_{i-1,s}^{W} * B_{W}^{j,s}
        Eigen::Isometry3d transform = body_frame_inv * to_transform(it->second.pose.data);
        auto pointcloud_i = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        // p_{i-1,s} = B_{i-1,s}^{j,s} * p_{j,s}
        pcl::transformPointCloudWithNormals(*it->second.pointcloud, *pointcloud_i, transform.matrix());
        *pointcloud += *pointcloud_i;
        if (++it == map.rend()) {
            break;
        }
    }
    return pointcloud;
}

const MapFrame& Mapping::last_frame() const {
    const auto it = map.crbegin();
    if (it == map.crend()) {
        throw std::runtime_error("Failed to get last map frame. Is the map empty()?");
    }
    return it->second;
}

bool Mapping::should_add_to_map(const eigen_ros::Pose& new_frame_pose, const eigen_ros::Pose& previous_frame_pose) {
    const double distance_m = (new_frame_pose.position - previous_frame_pose.position).norm();
    const double rotation_rad = new_frame_pose.orientation.angularDistance(previous_frame_pose.orientation);
    return distance_m > distance_threshold || rotation_rad > rotation_threshold;
}

}
