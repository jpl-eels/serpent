#include "eigen_ros/imu.hpp"

#include <iomanip>
#include <iostream>

namespace eigen_ros {

Imu::Imu(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity,
        const Eigen::Vector3d& linear_acceleration, const Eigen::Matrix3d& orientation_covariance,
        const Eigen::Matrix3d& angular_velocity_covariance, const Eigen::Matrix3d& inear_acceleration_covariance,
        const ros::Time& timestamp, const std::string& frame)
    : orientation(orientation),
      angular_velocity(angular_velocity),
      linear_acceleration(linear_acceleration),
      orientation_covariance(orientation_covariance),
      angular_velocity_covariance(angular_velocity_covariance),
      linear_acceleration_covariance(linear_acceleration_covariance),
      timestamp(timestamp),
      frame(frame) {}

void Imu::change_frame(const Eigen::Quaterniond& to_frame_ext, const Eigen::Quaterniond& from_frame_ext) {
    const Eigen::Matrix3d to_frame_ext_m = to_frame_ext.matrix();
    const Eigen::Matrix3d to_frame_ext_m_transpose = to_frame_ext_m.transpose();
    const Eigen::Matrix3d from_frame_ext_m = from_frame_ext.matrix();
    const Eigen::Matrix3d from_frame_ext_m_transpose = from_frame_ext_m.transpose();

    // Orientation
    orientation = orientation * to_frame_ext;
    orientation_covariance = to_frame_ext_m * orientation_covariance * to_frame_ext_m_transpose;

    // Angular velocity
    angular_velocity = from_frame_ext * angular_velocity;
    angular_velocity_covariance = from_frame_ext_m * angular_velocity_covariance * from_frame_ext_m_transpose;

    // Linear acceleration
    linear_acceleration = from_frame_ext * linear_acceleration;
    linear_acceleration_covariance = from_frame_ext_m * linear_acceleration_covariance * from_frame_ext_m_transpose;
}

void Imu::print(const unsigned int precision) const {
    Eigen::Vector3d rpy_ = rpy();
    std::stringstream ss;
    ss << std::setprecision(precision);
    ss << "IMU (" << timestamp << ", " << frame << ")\n"
       << "\torientation:\n"
       << "\t\tquaternion (wxyz): [" << orientation.w() << ", " << orientation.x() << ", " << orientation.y() << ", "
       << orientation.z() << "]\n"
       << "\t\tRPY (radians):     [" << rpy_[0] << ", " << rpy_[1] << ", " << rpy_[2] << "]\n"
       << "\t\tRPY (degrees):     [" << 180.0 * rpy_[0] / M_PI << ", " << 180.0 * rpy_[1] / M_PI << ", "
       << 180.0 * rpy_[2] / M_PI << "]\n"
       << "\tangular velocity:    [" << angular_velocity[0] << ", " << angular_velocity[1] << ", "
       << angular_velocity[2] << "] (" << angular_velocity.norm() << ")\n"
       << "\tlinear acceleration: [" << linear_acceleration[0] << ", " << linear_acceleration[1] << ", "
       << linear_acceleration[2] << "] (" << linear_acceleration.norm() << ")\n";
    std::cerr << ss.str();
}

bool operator==(const Imu& lhs, const Imu& rhs) {
    return lhs.orientation.toRotationMatrix().isApprox(rhs.orientation.toRotationMatrix()) &&
           lhs.angular_velocity.isApprox(rhs.angular_velocity) &&
           lhs.linear_acceleration.isApprox(rhs.linear_acceleration) &&
           lhs.orientation_covariance.isApprox(rhs.orientation_covariance) &&
           lhs.angular_velocity_covariance.isApprox(rhs.angular_velocity_covariance) &&
           lhs.linear_acceleration_covariance.isApprox(rhs.linear_acceleration_covariance) &&
           lhs.timestamp == rhs.timestamp && lhs.frame == rhs.frame;
}

}
