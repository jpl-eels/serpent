#include "eigen_ros/nav_msgs.hpp"

#include "eigen_ros/geometry_msgs.hpp"
#include "eigen_ros/std_msgs.hpp"

namespace eigen_ros {

void from_ros(const nav_msgs::Odometry& msg, Odometry& odom) {
    from_ros(msg.header, odom.timestamp, odom.frame);
    odom.child_frame = msg.child_frame_id;
    from_ros(msg.pose, odom.pose);
    from_ros(msg.twist, odom.twist);
}

void to_ros(nav_msgs::Odometry& msg, const Odometry& odom) {
    to_ros(msg.header, odom.timestamp, odom.frame);
    msg.child_frame_id = odom.child_frame;
    to_ros(msg.pose, odom.pose);
    to_ros(msg.twist, odom.twist);
}

}
