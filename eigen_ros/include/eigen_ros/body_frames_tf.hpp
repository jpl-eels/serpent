#ifndef EIGEN_ROS_BODY_FRAMES_TF_HPP
#define EIGEN_ROS_BODY_FRAMES_TF_HPP

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "eigen_ros/body_frames.hpp"

namespace eigen_ros {

/**
 * @brief Publish body frames as static tfs. Must be part of a ROS Node.
 *
 */
class BodyFramesTf {
public:
    explicit BodyFramesTf();

private:
    //// ROS Communications
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

    // Body frames
    const BodyFrames body_frames;
};

}

#endif
