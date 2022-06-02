#ifndef SERPENT_MAPPING_HPP
#define SERPENT_MAPPING_HPP

#include <eigen_ros/pose.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <map>
#include <mutex>

namespace serpent {

struct MapFrame {
    explicit MapFrame(const eigen_ros::PoseStamped& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud);

    eigen_ros::PoseStamped pose;
    pcl::PointCloud<pcl::PointNormal>::ConstPtr pointcloud;
};

class Mapping {
public:
    explicit Mapping();

private:
    /**
     * @brief Shortcut function to add a pointcloud to the map
     * 
     * @param pose_stamped 
     * @param pointcloud 
     * @return std::pair<std::map<ros::Time, MapFrame>::iterator, bool> 
     */
    std::pair<std::map<ros::Time, MapFrame>::iterator, bool> add_to_map(const eigen_ros::PoseStamped& pose_stamped,
            const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pointcloud);

    /**
     * @brief Correct the map according to a set of previous poses, then incorporate the new pointcloud into the map.
     * 
     * @param pointcloud_msg 
     * @param path_changes_msg 
     */
    void map_update_callback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pointcloud_msg,
            const nav_msgs::Path::ConstPtr& path_changes_msg);

    /**
     * @brief Generate a concatenated pointcloud consisting of the n last frames in the body frame.
     * 
     * @param n 
     * @param body_frame 
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr 
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr extract_past_frames(const std::size_t n,
            const eigen_ros::PoseStamped& body_frame);

    /**
     * @brief Shortcut to get the last frame in the map. Assumes map is sorted by timestamp. Throws std::runtime_error
     * if map is empty.
     * 
     * @return const MapFrame& 
     */
    const MapFrame& last_frame() const;

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
    message_filters::Subscriber<pcl::PointCloud<pcl::PointNormal>> pointcloud_subscriber;
    message_filters::Subscriber<nav_msgs::Path> path_changes_subscriber;
    message_filters::TimeSynchronizer<pcl::PointCloud<pcl::PointNormal>, nav_msgs::Path> correct_map_sync;

    //// Configuration
    // Translation threshold (metres)
    double distance_threshold;
    // Rotation threshold (radians)
    double rotation_threshold;
    // Frame extraction number
    std::size_t frame_extraction_number;

    // Map
    std::map<ros::Time, MapFrame> map;
};

}

#endif
