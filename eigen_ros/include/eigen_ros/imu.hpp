#ifndef EIGEN_ROS_IMU_HPP
#define EIGEN_ROS_IMU_HPP

#include <ros/time.h>

#include <Eigen/Geometry>
#include <string>

namespace eigen_ros {

Eigen::Vector3d rpy(const Eigen::Quaterniond& q);

struct Imu {
    Imu(const Eigen::Quaterniond& orientation = Eigen::Quaterniond(1, 0, 0, 0),
            const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero(),
            const Eigen::Vector3d& linear_acceleration = Eigen::Vector3d::Zero(),
            const Eigen::Matrix3d& orientation_covariance = Eigen::Matrix3d::Zero(),
            const Eigen::Matrix3d& angular_velocity_covariance = Eigen::Matrix3d::Zero(),
            const Eigen::Matrix3d& linear_acceleration_covariance = Eigen::Matrix3d::Zero(),
            const ros::Time& timestamp = ros::Time(0.0), const std::string& frame = std::string());

    /**
     * @brief Transform IMU to a new reference frame using a rotation extrinsic. The inverse transform must also be
     * passed to this function, which allows for greater efficiency when this transform can be calculated only once.
     *
     * TODO: The math on the orientation covariance needs to be checked.
     *
     * @param to_frame_ext rotation extrinsic from current (IMU) to new frame
     * @param from_frame_ext rotation extrinsic from new to current (IMU) frame (should equal inverse of to_frame_ext)
     */
    void change_frame(const Eigen::Quaterniond& to_frame_ext, const Eigen::Quaterniond& from_frame_ext);

    void print(const unsigned int precision = 3) const;

    Eigen::Vector3d rpy() const;

    // Orientation measurement. (0,0,0,0) if not known.
    Eigen::Quaterniond orientation;
    // Angular velocity measurement.
    Eigen::Vector3d angular_velocity;
    // Linear acceleration measurement.
    Eigen::Vector3d linear_acceleration;
    // Orientation covariance.
    Eigen::Matrix3d orientation_covariance;
    // Angular velocity covariance.
    Eigen::Matrix3d angular_velocity_covariance;
    // Linear acceleration covariance.
    Eigen::Matrix3d linear_acceleration_covariance;
    // Measurement timestamp.
    ros::Time timestamp;
    // Reference frame for measurement.
    std::string frame;
};

bool operator==(const Imu& lhs, const Imu& rhs);

/* Implementation ****************************************************************************************************/

inline Eigen::Vector3d rpy(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix().eulerAngles(0, 1, 2);
}

inline Eigen::Vector3d Imu::rpy() const {
    return eigen_ros::rpy(orientation);
}

}

#endif
