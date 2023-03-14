#ifndef SERPENT_MAPPING_HPP
#define SERPENT_MAPPING_HPP

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/pose.hpp>
#include <map>
#include <mutex>

namespace serpent {

template<typename PointT>
struct MapFrame {
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    explicit MapFrame(const eigen_ros::PoseStamped& pose, const PointCloudConstPtr pointcloud);

    // Origin of pointcloud with respect to the map/world, i.e. T_W^{L_i} where the pointcloud origin is at the lidar
    // frame L_i of this frame
    eigen_ros::PoseStamped pose;
    PointCloudConstPtr pointcloud;
};

template<typename PointT>
class Mapping {
public:
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    using Frame = MapFrame<PointT>;

    explicit Mapping();

private:
    /**
     * @brief Shortcut function to add a pointcloud to the map
     *
     * @param pose_stamped
     * @param pointcloud
     * @return std::pair<std::map<ros::Time, Frame>::iterator, bool>
     */
    std::pair<typename std::map<ros::Time, Frame>::iterator, bool> add_to_map(
            const eigen_ros::PoseStamped& pose_stamped, const PointCloudConstPtr& pointcloud);

    /**
     * @brief Generate a concatenated pointcloud consisting of the n last frames in the body frame.
     *
     * @param n
     * @param body_frame
     * @return PointCloud::Ptr
     */
    PointCloudPtr extract_past_frames(const std::size_t n, const eigen_ros::PoseStamped& body_frame);

    /**
     * @brief Shortcut to get the last frame in the map. Assumes map is sorted by timestamp. Throws std::runtime_error
     * if map is empty.
     *
     * @return const Frame&
     */
    const Frame& last_frame() const;

    /**
     * @brief Correct the map according to a set of previous poses, then incorporate the new pointcloud into the map.
     *
     * @param pointcloud_msg
     * @param path_changes_msg
     */
    void map_update_callback(const PointCloudConstPtr& pointcloud_msg,
            const nav_msgs::Path::ConstPtr& path_changes_msg);

    bool publish_map_service_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool publish_pose_graph_service_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    /**
     * @brief Check if a new frame should be added to the map.
     *
     * @param new_frame_pose New frame pose
     * @param previous_frame_pose Previous frame pose
     * @return true if the frame should be added
     * @return false otherwise
     */
    bool should_add_to_map(const eigen_ros::Pose& new_frame_pose, const eigen_ros::Pose& previous_frame_pose);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher local_map_publisher;
    ros::Publisher global_map_publisher;
    ros::Publisher pose_graph_publisher;

    message_filters::Subscriber<PointCloud> pointcloud_subscriber;
    message_filters::Subscriber<nav_msgs::Path> path_changes_subscriber;
    message_filters::TimeSynchronizer<PointCloud, nav_msgs::Path> correct_map_sync;
    ros::ServiceServer publish_map_server;
    ros::ServiceServer publish_pose_graph_server;

    //// Thread Management
    mutable std::mutex map_mutex;

    // Body frames
    const eigen_ros::BodyFrames body_frames;
    // Map frame id
    std::string map_frame_id;

    //// Configuration
    // Translation threshold (metres)
    double distance_threshold;
    // Rotation threshold (radians)
    double rotation_threshold;
    // Frame extraction number
    std::size_t frame_extraction_number;

    // Map
    std::map<ros::Time, Frame> map;

    // Downsampling
    bool voxel_grid_enabled;
    pcl::VoxelGrid<PointT> voxel_grid_filter;
};

}

#include "serpent/impl/mapping.hpp"

#endif
