#include "eigen_ros/body_frames_tf.hpp"

#include <geometry_msgs/TransformStamped.h>

#include "eigen_ros/geometry_msgs.hpp"

namespace eigen_ros {

BodyFramesTf::BodyFramesTf() {
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const std::string& frame : body_frames.frames()) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time(0);
        transform.header.frame_id = body_frames.body_frame_id();
        transform.child_frame_id = body_frames.frame_id(frame);
        eigen_ros::to_ros(transform.transform, body_frames.body_to_frame(frame));
        transforms.push_back(transform);
    }
    static_tf_broadcaster.sendTransform(transforms);
    ROS_INFO_STREAM("Published " << transforms.size() << " body frame static transforms.");
}

}
