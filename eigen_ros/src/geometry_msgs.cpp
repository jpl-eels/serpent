#include "eigen_ros/geometry_msgs.hpp"
#include "eigen_ros/eigen_ros.hpp"
#include <eigen_ext/covariance.hpp>

namespace eigen_ros {

void from_ros(const geometry_msgs::Point& msg, Eigen::Vector3d& v) {
    v[0] = msg.x;
    v[1] = msg.y;
    v[2] = msg.z;
}

void to_ros(geometry_msgs::Point& msg, const Eigen::Vector3d& v) {
    msg.x = v(0);
    msg.y = v(1);
    msg.z = v(2);
}

void from_ros(const geometry_msgs::PoseWithCovarianceStamped& msg, PoseStamped& pose) {
    pose.timestamp = msg.header.stamp;
    from_ros(msg.pose, pose.data);
}

void to_ros(geometry_msgs::PoseWithCovarianceStamped& msg, const PoseStamped& pose) {
    msg.header.stamp = pose.timestamp;
    to_ros(msg.pose, pose.data);
}

void from_ros(const geometry_msgs::PoseWithCovariance& msg, Pose& pose) {
    from_ros(msg.pose, pose);
    from_ros(msg.covariance, pose.covariance);
    pose.covariance = eigen_ext::reorder_covariance(pose.covariance, 3);
}

void to_ros(geometry_msgs::PoseWithCovariance& msg, const Pose& pose) {
    to_ros(msg.pose, pose);
    to_ros(msg.covariance, eigen_ext::reorder_covariance(pose.covariance, 3));
}

void from_ros(const geometry_msgs::PoseStamped& msg, PoseStamped& pose) {
    pose.timestamp = msg.header.stamp;
    from_ros(msg.pose, pose.data);
}

void to_ros(geometry_msgs::PoseStamped& msg, const PoseStamped& pose) {
    msg.header.stamp = pose.timestamp;
    to_ros(msg.pose, pose.data);
}

void from_ros(const geometry_msgs::Pose& msg, Pose& pose) {
    from_ros(msg.position, pose.position);
    from_ros(msg.orientation, pose.orientation);
}

void to_ros(geometry_msgs::Pose& msg, const Pose& pose) {
    to_ros(msg.position, pose.position);
    to_ros(msg.orientation, pose.orientation);
}

void from_ros(const geometry_msgs::Vector3& p, const geometry_msgs::Quaternion& q, Eigen::Matrix4d& matrix) {
    matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d p_;
    from_ros(p, p_);
    matrix.block<3, 1>(0, 3) = p_;
    Eigen::Quaterniond q_;
    from_ros(q, q_);
    matrix.block<3, 3>(0, 0) = q_.matrix();
}

void from_ros(const geometry_msgs::Point& p, const geometry_msgs::Quaternion& q, Eigen::Matrix4d& matrix) {
    matrix = Eigen::Matrix4d::Identity();
    Eigen::Vector3d p_;
    from_ros(p, p_);
    matrix.block<3, 1>(0, 3) = p_;
    Eigen::Quaterniond q_;
    from_ros(q, q_);
    matrix.block<3, 3>(0, 0) = q_.matrix();
}

void from_ros(const geometry_msgs::Pose& msg, Eigen::Matrix4d& matrix) {
    from_ros(msg.position, msg.orientation, matrix);
}

void to_ros(geometry_msgs::Pose& msg, const Eigen::Matrix4d& matrix) {
    to_ros(msg.position, matrix.block<3, 1>(0, 3));
    to_ros(msg.orientation, matrix.block<3, 3>(0, 0));
}

void from_ros(const geometry_msgs::TransformStamped& msg, PoseStamped& pose) {
    pose.timestamp = msg.header.stamp;
    from_ros(msg.transform, pose.data);
}

void to_ros(geometry_msgs::TransformStamped& msg, const PoseStamped& pose) {
    msg.header.stamp = pose.timestamp;
    to_ros(msg.transform, pose.data);
}

void from_ros(const geometry_msgs::Transform& msg, Pose& pose) {
    from_ros(msg.translation, pose.position);
    from_ros(msg.rotation, pose.orientation);
}

void to_ros(geometry_msgs::Transform& msg, const Pose& pose) {
    to_ros(msg.translation, pose.position);
    to_ros(msg.rotation, pose.orientation);
}

void from_ros(const geometry_msgs::Transform& msg, Eigen::Isometry3d& transform) {
    eigen_ros::Pose pose;
    from_ros(msg, pose);
    transform = to_transform(pose);
}

void to_ros(geometry_msgs::Transform& msg, const Eigen::Isometry3d& transform) {
    to_ros(msg.translation, transform.translation());
    to_ros(msg.rotation, Eigen::Quaterniond(transform.rotation()));
}

void from_ros(const geometry_msgs::Transform& msg, Eigen::Matrix4d& matrix) {
    from_ros(msg.translation, msg.rotation, matrix);
}

void to_ros(geometry_msgs::Transform& msg, const Eigen::Matrix4d& matrix) {
    to_ros(msg.translation, matrix.block<3, 1>(0, 3));
    to_ros(msg.rotation, matrix.block<3, 3>(0, 0));
}

void from_ros(const geometry_msgs::Quaternion& msg, Eigen::Quaterniond& q) {
    q.w() = msg.w;
    q.x() = msg.x;
    q.y() = msg.y;
    q.z() = msg.z;
}

void to_ros(geometry_msgs::Quaternion& msg, const Eigen::Quaterniond& q) {
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
}

void from_ros(const geometry_msgs::Quaternion& msg, Eigen::Matrix3d& matrix) {
    Eigen::Quaterniond q;
    from_ros(msg, q);
    matrix = q.matrix();
}

void to_ros(geometry_msgs::Quaternion& msg, const Eigen::Matrix3d& matrix) {
    Eigen::Quaterniond q{matrix};
    to_ros(msg, q);
}

void from_ros(const geometry_msgs::TwistWithCovariance& msg, Twist& twist) {
    from_ros(msg.twist, twist);
    from_ros(msg.covariance, twist.covariance);
    twist.covariance = eigen_ext::reorder_covariance(twist.covariance, 3);
}

void to_ros(geometry_msgs::TwistWithCovariance& msg, const Twist& twist) {
    to_ros(msg.twist, twist);
    to_ros(msg.covariance, eigen_ext::reorder_covariance(twist.covariance, 3));
}

void from_ros(const geometry_msgs::Twist& msg, Twist& twist) {
    from_ros(msg.linear, twist.linear);
    from_ros(msg.angular, twist.angular);
}

void to_ros(geometry_msgs::Twist& msg, const Twist& twist) {
    to_ros(msg.linear, twist.linear);
    to_ros(msg.angular, twist.angular);
}

void from_ros(const geometry_msgs::Vector3& msg, Eigen::Vector3d& v) {
    v[0] = msg.x;
    v[1] = msg.y;
    v[2] = msg.z;
}

void to_ros(geometry_msgs::Vector3& msg, const Eigen::Vector3d& v) {
    msg.x = v(0);
    msg.y = v(1);
    msg.z = v(2);
}

}
