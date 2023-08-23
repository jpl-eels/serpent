#ifndef SERPENT_IMPL_MAPPING_HPP
#define SERPENT_IMPL_MAPPING_HPP

#include <pcl/common/transforms.h>

#include <cmath>
#include <eigen_ros/eigen_ros.hpp>
#include <eigen_ros/geometry_msgs.hpp>

#include "serpent/PoseGraph.h"
#include "serpent/front_end/lidar/transform_pointcloud.hpp"
#include "serpent/mapping/mapping.hpp"

namespace serpent {

FrameExtractionMethod to_frame_extraction_method(const std::string& method) {
    if (method == "NEARBY") {
        return FrameExtractionMethod::NEARBY;
    } else if (method == "PAST") {
        return FrameExtractionMethod::PAST;
    } else {
        throw std::runtime_error("FrameExtractionMethod \"" + method + "\" not recognised.");
    }
}

template<typename PointT>
MapFrame<PointT>::MapFrame(const eigen_ros::PoseStamped& pose, const PointCloudConstPtr pointcloud)
    : pose(pose), pointcloud(pointcloud) {}

template<typename PointT>
typename MapFrame<PointT>::PointCloudPtr MapFrame<PointT>::transformed_pointcloud(
        const Eigen::Isometry3d& target_lidar_frame_inv) const {
    // T_{L_t}^{L_j} = T_{L_t}^W * T_W^{L_j}
    const Eigen::Isometry3d transform = target_lidar_frame_inv * to_transform(pose.data);
    PointCloudPtr pointcloud_target = boost::make_shared<PointCloud>();
    // p_{L_t} = T_{L_t}^{L_j} * p_{L_j}
    transform_pointcloud(*pointcloud, *pointcloud_target, transform);
    return pointcloud_target;
}

template<typename PointT>
Mapping<PointT>::Mapping() : nh("serpent"), correct_map_sync(10), body_frames("serpent") {
    // Publishers
    local_map_publisher = nh.advertise<PointCloud>("mapping/local_map", 1);
    global_map_publisher = nh.advertise<PointCloud>("output/global_map", 1, true);
    pose_graph_publisher = nh.advertise<serpent::PoseGraph>("output/pose_graph", 1, true);

    // Subscribers
    pointcloud_subscriber.subscribe(nh, "normal_estimation/pointcloud", 10);
    path_changes_subscriber.subscribe(nh, "optimisation/path_changes", 10);
    correct_map_sync.connectInput(pointcloud_subscriber, path_changes_subscriber);
    correct_map_sync.registerCallback(boost::bind(&Mapping<PointT>::map_update_callback, this, _1, _2));

    // Service servers
    publish_map_server = nh.advertiseService("publish_map", &Mapping<PointT>::publish_map_service_callback, this);
    publish_pose_graph_server =
            nh.advertiseService("publish_pose_graph", &Mapping<PointT>::publish_pose_graph_service_callback, this);

    // Configuration
    nh.param<std::string>("map_frame_id", map_frame_id, "map");
    nh.param<double>("mapping/distance_threshold", distance_threshold, 1.0);
    nh.param<double>("mapping/rotation_threshold", rotation_threshold, M_PI / 6.0);
    const int frame_extraction_number_ = nh.param<int>("mapping/frame_extraction_number", 10);
    if (frame_extraction_number_ < 1) {
        throw std::runtime_error("Frame extraction number " + std::to_string(frame_extraction_number_) + " invalid");
    }
    auto frame_extraction_method =
            to_frame_extraction_method(nh.param<std::string>("mapping/frame_extraction_method", "NEARBY"));
    switch (frame_extraction_method) {
        case FrameExtractionMethod::NEARBY:
            frame_extraction_function = &Mapping<PointT>::extract_nearby_frames;
            pose_cloud = boost::make_shared<pcl::PointCloud<PointAngleAxis>>();
            nearby_search = boost::make_shared<pcl::search::KdTree<PointAngleAxis>>();
            break;
        case FrameExtractionMethod::PAST:
            frame_extraction_function = &Mapping<PointT>::extract_past_frames;
            break;
        default:
            throw std::runtime_error("FrameExtractionMethod not handled.");
    }
    frame_extraction_number = static_cast<std::size_t>(frame_extraction_number_);

    // Voxel grid configuration
    voxel_grid_enabled_local = nh.param<bool>("mapping/voxel_grid_filter/enabled_local", true);
    voxel_grid_enabled_global = nh.param<bool>("mapping/voxel_grid_filter/enabled_global", true);
    const double leaf_size = nh.param<double>("mapping/voxel_grid_filter/leaf_size", 0.05);
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.setMinimumPointsNumberPerVoxel(
            nh.param<double>("mapping/voxel_grid_filter/minimum_points_number_per_voxel", 1));
}

template<typename PointT>
std::pair<typename std::map<ros::Time, typename Mapping<PointT>::Frame>::iterator, bool> Mapping<PointT>::add_to_map(
        const eigen_ros::PoseStamped& pose_stamped, const PointCloudConstPtr& pointcloud) {
    if (pose_cloud) {
        pose_cloud_indices.emplace(pose_stamped.timestamp, pose_cloud->size());
        pose_cloud_timestamps.emplace(pose_cloud->size(), pose_stamped.timestamp);
        pose_cloud->push_back(
                PointAngleAxis{pose_stamped.data.position.cast<float>(), pose_stamped.data.orientation.cast<float>()});
    }
    return map.emplace(pose_stamped.timestamp, Frame{pose_stamped, pointcloud});
}

template<typename PointT>
typename Mapping<PointT>::PointCloudPtr Mapping<PointT>::extract_nearby_frames(
        const std::size_t n, const eigen_ros::PoseStamped& lidar_frame) {
    // Use the kd-tree to find the closest n poses
    nearby_search->setInputCloud(pose_cloud);
    const std::size_t k = std::min(n, pose_cloud->size());
    std::vector<int> knn_indices(k);
    std::vector<float> knn_square_distances(k);
    PointAngleAxis search_point{lidar_frame.data.position.cast<float>(), lidar_frame.data.orientation.cast<float>()};
    auto pointcloud = boost::make_shared<PointCloud>();
    pointcloud->header.stamp = pcl_conversions::toPCL(lidar_frame.timestamp);
    pointcloud->header.frame_id = body_frames.frame_id("lidar");
    if (nearby_search->nearestKSearch(search_point, k, knn_indices, knn_square_distances) == k) {
        // L_{i-1} is the target frame: T_{L_{i-1}}^W = (T_W^{L_{i-1}})^-1
        const Eigen::Isometry3d lidar_frame_inv = to_transform(lidar_frame.data).inverse();
        for (const int index : knn_indices) {
            const Frame& map_frame = map.at(pose_cloud_timestamps.at(static_cast<std::size_t>(index)));
            if (map_frame.pose.timestamp > lidar_frame.timestamp) {
                throw std::runtime_error("Timestamp in the future detected. Something went wrong.");
            }
            auto pointcloud_target = map_frame.transformed_pointcloud(lidar_frame_inv);
            *pointcloud += *pointcloud_target;
        }
    } else {
        throw std::runtime_error("Failed to find k = " + std::to_string(k) + " nearest neighbour(s).");
    }
    return pointcloud;
}

template<typename PointT>
typename Mapping<PointT>::PointCloudPtr Mapping<PointT>::extract_past_frames(
        const std::size_t n, const eigen_ros::PoseStamped& lidar_frame) {
    // Starting from the end, find the first pointcloud to merge
    auto it = map.rbegin();
    while (it->first > lidar_frame.timestamp) {
        ++it;
    }
    auto pointcloud = boost::make_shared<PointCloud>();
    pointcloud->header.stamp = pcl_conversions::toPCL(lidar_frame.timestamp);
    pointcloud->header.frame_id = body_frames.frame_id("lidar");
    // L_{i-1} is the target frame: T_{L_{i-1}}^W = (T_W^{L_{i-1}})^-1
    const Eigen::Isometry3d lidar_frame_inv = to_transform(lidar_frame.data).inverse();
    for (std::size_t i = 0; i < n; ++i) {
        const Frame& map_frame = it->second;
        if (map_frame.pose.timestamp > lidar_frame.timestamp) {
            throw std::runtime_error("Timestamp in the future detected. Something went wrong.");
        }
        auto pointcloud_target = map_frame.transformed_pointcloud(lidar_frame_inv);
        *pointcloud += *pointcloud_target;
        if (++it == map.rend()) {
            break;
        }
    }
    return pointcloud;
}

template<typename PointT>
typename Mapping<PointT>::PointCloudPtr Mapping<PointT>::filter_pointcloud(const PointCloudConstPtr pointcloud) {
    voxel_grid_filter.setInputCloud(pointcloud);
    auto filtered_pointcloud = boost::make_shared<PointCloud>();
    voxel_grid_filter.filter(*filtered_pointcloud);
    return filtered_pointcloud;
}

template<typename PointT>
const typename Mapping<PointT>::Frame& Mapping<PointT>::last_frame() const {
    const auto it = map.crbegin();
    if (it == map.crend()) {
        throw std::runtime_error("Failed to get last map frame. Is the map empty()?");
    }
    return it->second;
}

template<typename PointT>
void Mapping<PointT>::map_update_callback(
        const PointCloudConstPtr& pointcloud_msg, const nav_msgs::Path::ConstPtr& path_changes_msg) {
    std::lock_guard<std::mutex> guard{map_mutex};
    const ros::WallTime tic = ros::WallTime::now();

    // Error handling
    if (path_changes_msg->poses.empty()) {
        throw std::runtime_error("path.poses was empty. Something went wrong.");
    }

    // Iterate over the path:
    // 1. Correct all pointclouds in the global map with matching timestamps
    // 2. Add the new pointcloud (and retain the pointcloud's pose)
    bool pointcloud_timestamp_found{false};
    const ros::Time pointcloud_timestamp = pcl_conversions::fromPCL(pointcloud_msg->header.stamp);
    eigen_ros::PoseStamped pointcloud_lidar_pose;
    for (const auto& pose : path_changes_msg->poses) {
        // Pose in the body frame (from optimisation)
        const eigen_ros::PoseStamped body_pose_stamped = eigen_ros::from_ros<eigen_ros::PoseStamped>(pose);
        // Convert to the lidar frame: T_W^{L_i} = T_W^{B_i} * T_{B_i}^{L_i}
        const Eigen::Isometry3d lidar_pose = to_transform(body_pose_stamped.data) * body_frames.body_to_frame("lidar");
        const eigen_ros::PoseStamped lidar_pose_stamped =
                eigen_ros::PoseStamped{eigen_ros::Pose{lidar_pose}, body_pose_stamped.timestamp};

        if (lidar_pose_stamped.timestamp == pointcloud_timestamp) {
            // Add new pointcloud (at last path pose) into map
            if (map.empty() || should_add_to_map(lidar_pose_stamped.data, last_frame().pose.data)) {
                add_to_map(lidar_pose_stamped, pointcloud_msg);
                ROS_INFO_STREAM(
                        "Added frame at " << pointcloud_timestamp << " to map (" << map.size() << " total keyclouds)");
            }
            pointcloud_lidar_pose = lidar_pose_stamped;
            pointcloud_timestamp_found = true;
        } else {
            update_map(lidar_pose_stamped);
        }
    }
    // Pointcloud's pose must be in the path
    if (!pointcloud_timestamp_found) {
        throw std::runtime_error("Pointcloud timestamp (" + std::to_string(pointcloud_timestamp.toSec()) +
                                 ") was not in the path poses. Something went wrong.");
    }

    // Extract map
    auto map_region = std::invoke(frame_extraction_function, this, frame_extraction_number, pointcloud_lidar_pose);

    // Filtering/Downsampling
    PointCloudPtr filtered_map_region = voxel_grid_enabled_local ? filter_pointcloud(map_region) : map_region;

    // Publish the local map
    local_map_publisher.publish(filtered_map_region);
    ROS_INFO_STREAM("Created and published local map in " << (ros::WallTime::now() - tic).toSec() << " seconds.");
}

template<typename PointT>
bool Mapping<PointT>::publish_pose_graph_service_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    std::lock_guard<std::mutex> guard{map_mutex};
    const ros::WallTime tic = ros::WallTime::now();

    // Construct pose graph
    serpent::PoseGraph pose_graph;
    for (const auto& map_frame_pair : map) {
        const auto& map_frame = map_frame_pair.second;
        sensor_msgs::PointCloud2 pointcloud_ros;
        pcl::toROSMsg(*map_frame.pointcloud, pointcloud_ros);
        pose_graph.clouds.push_back(pointcloud_ros);
        auto pose = eigen_ros::to_ros<geometry_msgs::PoseWithCovarianceStamped>(map_frame.pose);
        pose.header.frame_id = body_frames.frame_id("lidar");
        pose_graph.poses.push_back(pose);
    }

    // Publish
    pose_graph_publisher.publish(pose_graph);
    ROS_INFO_STREAM("Created and published pose graph in " << (ros::WallTime::now() - tic).toSec() << " seconds.");
    return true;
}

template<typename PointT>
bool Mapping<PointT>::publish_map_service_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    std::lock_guard<std::mutex> guard{map_mutex};
    const ros::WallTime tic = ros::WallTime::now();

    // Assemble pointcloud
    auto pointcloud = boost::make_shared<PointCloud>();
    if (!map.empty()) {
        pointcloud->header.stamp =
                pcl_conversions::toPCL(ros::Time::now());  // pcl_conversions::toPCL(last_frame().pose.timestamp);
        pointcloud->header.frame_id = map_frame_id;
        for (const auto& map_frame_pair : map) {
            const auto& map_frame = map_frame_pair.second;
            auto pointcloud_i = boost::make_shared<PointCloud>();
            // p_W = T_W^{L_i} * p_{L_i}
            transform_pointcloud(*map_frame.pointcloud, *pointcloud_i, eigen_ros::to_transform(map_frame.pose.data));
            *pointcloud += *pointcloud_i;
        }
    }

    // Filtering/Downsampling
    PointCloudPtr filtered_pointcloud = voxel_grid_enabled_local ? filter_pointcloud(pointcloud) : pointcloud;

    // Publish
    global_map_publisher.publish(filtered_pointcloud);
    ROS_INFO_STREAM("Created and published global map in " << (ros::WallTime::now() - tic).toSec() << " seconds.");
    return true;
}

template<typename PointT>
bool Mapping<PointT>::should_add_to_map(
        const eigen_ros::Pose& new_frame_pose, const eigen_ros::Pose& previous_frame_pose) {
    const double distance_m = (new_frame_pose.position - previous_frame_pose.position).norm();
    const double rotation_rad = new_frame_pose.orientation.angularDistance(previous_frame_pose.orientation);
    return distance_m > distance_threshold || rotation_rad > rotation_threshold;
}

template<typename PointT>
void Mapping<PointT>::update_map(const eigen_ros::PoseStamped& frame_pose_stamped) {
    // Update pose in the map
    auto it = map.find(frame_pose_stamped.timestamp);
    if (it != map.end()) {
        it->second.pose = frame_pose_stamped;
        pose_cloud->at(pose_cloud_indices.at(it->second.pose.timestamp)) = PointAngleAxis{
                frame_pose_stamped.data.position.cast<float>(), frame_pose_stamped.data.orientation.cast<float>()};
    }
}

}

#endif
