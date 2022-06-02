#ifndef EIGEN_ROS_STAMPED_HPP
#define EIGEN_ROS_STAMPED_HPP

#include <ros/ros.h>

namespace eigen_ros {

template<typename T>
class Stamped {
public:
    explicit Stamped(const T& data = T(), const ros::Time& timestamp = ros::Time());

    ros::Time timestamp;
    T data;
};

template<typename T>
bool operator==(const Stamped<T>& lhs, const Stamped<T>& rhs);

/* Implementation ****************************************************************************************************/

template<typename T>
Stamped<T>::Stamped(const T& data, const ros::Time& timestamp):
    data(data), timestamp(timestamp) {}

template<typename T>
bool operator==(const Stamped<T>& lhs, const Stamped<T>& rhs) {
    return lhs.timestamp == rhs.timestamp && lhs.data == rhs.data;
}

}

#endif
