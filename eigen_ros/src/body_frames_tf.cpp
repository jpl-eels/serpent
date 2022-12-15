#include "eigen_ros/body_frames_tf.hpp"

#include <geometry_msgs/TransformStamped.h>

#include "eigen_ros/geometry_msgs.hpp"

namespace eigen_ros {

BodyFramesTf::BodyFramesTf()
    : nh("~") {
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const std::string& frame : body_frames.frames()) {
        const std::string frame_id = body_frames.frame_id(frame);
        if (frame_id != body_frames.body_frame_id() && !body_frames.lookedup_tf(frame)) {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time(0);
            transform.header.frame_id = body_frames.body_frame_id();
            transform.child_frame_id = frame_id;
            eigen_ros::to_ros(transform.transform, body_frames.body_to_frame(frame));
            transforms.push_back(transform);
        }
        const std::vector<std::string>& aliases = body_frames.aliases(frame_id);
        for (const auto& alias : aliases) {
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time(0);
            transform.header.frame_id = frame_id;
            transform.child_frame_id = alias;
            eigen_ros::to_ros(transform.transform, Eigen::Isometry3d::Identity());
            transforms.push_back(transform);
        }
    }
    static_tf_broadcaster.sendTransform(transforms);
    ROS_INFO_STREAM("Loaded " << body_frames.frames().size() << " frames and published " << transforms.size()
                              << " static transforms.");

    // Create lookup transform server
    lookup_transform_server = nh.advertiseService("lookup_transform", &BodyFramesTf::lookup_transform_callback, this);
}

bool BodyFramesTf::lookup_transform_callback(eigen_ros::lookup_transform::Request& request,
        eigen_ros::lookup_transform::Response& response) {
    eigen_ros::to_ros(response.transform, body_frames.frame_to_frame(request.source_frame, request.target_frame));
    return true;
}

}
