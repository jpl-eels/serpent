#include "eigen_ros/std_msgs.hpp"

namespace eigen_ros {

void from_ros(const std_msgs::Header& msg, ros::Time& timestamp, std::string& frame_id) {
    timestamp = msg.stamp;
    frame_id = msg.frame_id;
}

void to_ros(std_msgs::Header& msg, const ros::Time& timestamp, const std::string& frame_id) {
    msg.stamp = timestamp;
    msg.frame_id = frame_id;
}

}
