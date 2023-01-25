#include "eigen_ros/imu.hpp"

#include <iomanip>
#include <iostream>

#include "eigen_ext/matrix.hpp"

namespace eigen_ros {

Imu::Imu(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity,
        const Eigen::Vector3d& linear_acceleration, const Eigen::Matrix3d& orientation_covariance,
        const Eigen::Matrix3d& angular_velocity_covariance, const Eigen::Matrix3d& linear_acceleration_covariance,
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

Imu interpolate(const Imu& imu1, const Imu& imu2, const ros::Time& interp_timestamp) {
    // Check interpolation is possible
    if (interp_timestamp < imu1.timestamp || interp_timestamp > imu2.timestamp) {
        throw std::runtime_error("Imu interpolation error: new_timestamp " + std::to_string(interp_timestamp.toSec()) +
                                 " was not between imu timestamps " + std::to_string(imu1.timestamp.toSec()) + " and " +
                                 std::to_string(imu2.timestamp.toSec()) + ".");
    }

    // Special case
    if (imu1.timestamp == imu2.timestamp) {
        return imu1;
    }

    // Interpolation constant
    const double interp = (interp_timestamp - imu1.timestamp).toSec() / (imu2.timestamp - imu1.timestamp).toSec();

    // Return interpolated Imu measurement.
    return Imu{imu1.orientation == Eigen::Quaterniond(0, 0, 0, 0) || imu2.orientation == Eigen::Quaterniond(0, 0, 0, 0)
                       ? Eigen::Quaterniond(0, 0, 0, 0)
                       : imu1.orientation.slerp(interp, imu2.orientation),
            eigen_ext::linear_interpolate(imu1.angular_velocity, imu2.angular_velocity, interp),
            eigen_ext::linear_interpolate(imu1.linear_acceleration, imu2.linear_acceleration, interp),
            interp <= 0.5 ? imu1.orientation_covariance : imu2.orientation_covariance,
            interp <= 0.5 ? imu1.angular_velocity_covariance : imu2.angular_velocity_covariance,
            interp <= 0.5 ? imu1.linear_acceleration_covariance : imu2.linear_acceleration_covariance, interp_timestamp,
            interp <= 0.5 ? imu1.frame : imu2.frame};
}

}
