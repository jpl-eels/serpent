#ifndef SERPENT_MAPPING_HPP
#define SERPENT_MAPPING_HPP

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <eigen_ros/body_frames.hpp>
#include <eigen_ros/pose.hpp>
#include <map>
#include <mutex>
#include <pointcloud_tools/point_types.hpp>

namespace serpent {

enum class FrameExtractionMethod {
    NEARBY,  // spatial
    PAST     // temporal
};

FrameExtractionMethod to_frame_extraction_method(const std::string& method);

template<typename PointT>
struct MapFrame {
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    explicit MapFrame(const eigen_ros::PoseStamped& pose, const PointCloudConstPtr pointcloud);

    /**
     * @brief Given the inverse trasform to the target frame {L_t}: T_{L_t}^W, transform and return this map frame's
     * pointcloud in the {L_t} frame.
     *
     * @param target_lidar_frame_inv
     * @return PointCloudPtr
     */
    PointCloudPtr transformed_pointcloud(const Eigen::Isometry3d& target_lidar_frame_inv) const;

    // Origin of pointcloud with respect to the map/world, i.e. T_W^{L_i} where the pointcloud origin is at the lidar
    // frame L_i of this frame
    eigen_ros::PoseStamped pose;
    // Pointcloud
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
    using FrameExtractionFunction = PointCloudPtr (
            Mapping<PointT>::*)(const std::size_t, const eigen_ros::PoseStamped&);

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
     * @brief Generate a concatenatned pointcloud consisting of the closest n frames to the lidar frame.
     *
     * @param n
     * @param body_frame
     * @return PointCloudPtr
     */
    PointCloudPtr extract_nearby_frames(const std::size_t n, const eigen_ros::PoseStamped& lidar_frame);

    /**
     * @brief Generate a concatenated pointcloud consisting of the n last frames in the lidar frame.
     *
     * @param n
     * @param body_frame
     * @return PointCloud::Ptr
     */
    PointCloudPtr extract_past_frames(const std::size_t n, const eigen_ros::PoseStamped& lidar_frame);

    PointCloudPtr filter_pointcloud(const PointCloudConstPtr pointcloud);

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

    void update_map(const eigen_ros::PoseStamped& frame_pose_stamped);

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
    // Frame extraction function
    FrameExtractionMethod frame_extraction_method;
    FrameExtractionFunction frame_extraction_function;
    // Frame extraction number
    std::size_t frame_extraction_number;

    //// Map
    // Mapframes
    std::map<ros::Time, Frame> map;
    // Pose cloud for nearby search
    pcl::PointCloud<PointAngleAxis>::Ptr pose_cloud;
    // Link between map frames and pose cloud. Maps map frame timestamps to pose_cloud indices
    std::map<ros::Time, std::size_t> pose_cloud_indices;
    std::map<std::size_t, ros::Time> pose_cloud_timestamps;

    // Kdtree for nearby search
    pcl::search::KdTree<PointAngleAxis>::Ptr nearby_search;

    // Downsampling
    bool voxel_grid_enabled_local;
    bool voxel_grid_enabled_global;
    pcl::VoxelGrid<PointT> voxel_grid_filter;
};

}

#include "serpent/impl/mapping.hpp"

#endif
