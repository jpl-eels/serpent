#include "eigen_ros/odometry.hpp"

namespace eigen_ros {

Odometry::Odometry(const Pose& pose, const Twist& twist, const ros::Time& timestamp, const std::string& frame,
        const std::string& child_frame):
    pose(pose), twist(twist), timestamp(timestamp), frame(frame), child_frame(child_frame) {}

bool operator==(const Odometry& lhs, const Odometry& rhs) {
    return lhs.pose == rhs.pose && lhs.twist == rhs.twist && lhs.timestamp == rhs.timestamp && lhs.frame == rhs.frame &&
            lhs.child_frame == rhs.child_frame;
}

}
