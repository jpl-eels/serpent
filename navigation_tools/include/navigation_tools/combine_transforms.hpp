#ifndef NAVIGATION_TOOLS_COMBINE_TRANSFORMS_HPP
#define NAVIGATION_TOOLS_COMBINE_TRANSFORMS_HPP

#include <eigen_ros/eigen_ros.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

class CombineTransforms {
public:
    explicit CombineTransforms();

private:
    /**
     * @brief Apply transform to current state, i.e. new pose T_{W}^{B_i} = T_{W}^{B_i-1} * T_{B_i-1}^{B_i}
     * 
     * @tparam T 
     * @param msg 
     */
    template<typename T>
    void combine_callback(const typename T::ConstPtr& msg);

    void combine(const eigen_ros::PoseStamped& transform);

    //// ROS Communication
    // Node handle
    ros::NodeHandle nh;
    // Odometry publisher
    ros::Publisher odometry_publisher;
    // Path publisher
    ros::Publisher path_publisher;
    // Transform subscriber
    ros::Subscriber subscriber;

    // State
    eigen_ros::Odometry odometry;
    // Path
    nav_msgs::Path path;
};

template<typename T>
void CombineTransforms::combine_callback(const typename T::ConstPtr& msg) {
    // Convert from ROS
    auto transform = eigen_ros::from_ros<eigen_ros::PoseStamped>(*msg);

    // Combine
    combine(transform);
}

#endif
