#include "eigen_ros/sensor_msgs.hpp"

#include "eigen_ros/eigen_ros.hpp"
#include "eigen_ros/geometry_msgs.hpp"
#include "eigen_ros/std_msgs.hpp"

namespace eigen_ros {

void from_ros(const sensor_msgs::Imu& msg, Imu& imu) {
    from_ros(msg.header, imu.timestamp, imu.frame);
    from_ros(msg.orientation, imu.orientation);
    from_ros(msg.angular_velocity, imu.angular_velocity);
    from_ros(msg.linear_acceleration, imu.linear_acceleration);
    from_ros(msg.orientation_covariance, imu.orientation_covariance);
    from_ros(msg.angular_velocity_covariance, imu.angular_velocity_covariance);
    from_ros(msg.linear_acceleration_covariance, imu.linear_acceleration_covariance);
}

void to_ros(sensor_msgs::Imu& msg, const Imu& imu) {
    to_ros(msg.header, imu.timestamp, imu.frame);
    to_ros(msg.orientation, imu.orientation);
    to_ros(msg.angular_velocity, imu.angular_velocity);
    to_ros(msg.linear_acceleration, imu.linear_acceleration);
    to_ros(msg.orientation_covariance, imu.orientation_covariance);
    to_ros(msg.angular_velocity_covariance, imu.angular_velocity_covariance);
    to_ros(msg.linear_acceleration_covariance, imu.linear_acceleration_covariance);
}

}
