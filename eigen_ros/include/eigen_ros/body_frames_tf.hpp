#ifndef EIGEN_ROS_BODY_FRAMES_TF_HPP
#define EIGEN_ROS_BODY_FRAMES_TF_HPP

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "eigen_ros/body_frames.hpp"
#include "eigen_ros/lookup_transform.h"

namespace eigen_ros {

/**
 * @brief Publish body frames as static tfs. Must be part of a ROS Node.
 *
 */
class BodyFramesTf {
public:
    explicit BodyFramesTf(const std::string& node_namespace = "~");

private:
    bool lookup_transform_callback(eigen_ros::lookup_transform::Request& request,
            eigen_ros::lookup_transform::Response& response);

    //// ROS Communications
    // Nodehandle
    ros::NodeHandle nh;
    // Static transform publisher
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    // Transform lookup server
    ros::ServiceServer lookup_transform_server;

    // Body frames
    const BodyFrames body_frames;
};

}

#endif
