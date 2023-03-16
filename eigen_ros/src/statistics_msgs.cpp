#include "eigen_ros/statistics_msgs.hpp"

#include "eigen_ros/eigen_ros.hpp"

namespace eigen_ros {

void from_ros(const statistics_msgs::Covariance& msg, Eigen::MatrixXd& covariance) {
    covariance.resize(msg.dimensions, msg.dimensions);
    from_ros(msg.matrix, covariance);
}

void from_ros(const statistics_msgs::CovarianceStamped& msg, Eigen::MatrixXd& covariance, ros::Time& timestamp,
        std::string& frame_id) {
    from_ros(msg.header, timestamp, frame_id);
    from_ros(msg.covariance, covariance);
}

void from_ros(const statistics_msgs::CovarianceFloat32& msg, Eigen::MatrixXf& covariance) {
    covariance.resize(msg.dimensions, msg.dimensions);
    from_ros(msg.matrix, covariance);
}

void from_ros(const statistics_msgs::CovarianceFloat32Stamped& msg, Eigen::MatrixXf& covariance, ros::Time& timestamp,
        std::string& frame_id) {
    from_ros(msg.header, timestamp, frame_id);
    from_ros(msg.covariance, covariance);
}

void to_ros(statistics_msgs::Covariance& msg, const Eigen::MatrixXd& covariance) {
    if (covariance.rows() != covariance.cols()) {
        throw std::runtime_error("Cannot convert covariance to ros. Matrix is not square, but " +
                                 std::to_string(covariance.rows()) + " by " + std::to_string(covariance.cols()));
    }
    msg.dimensions = covariance.rows();
    to_ros(msg.matrix, covariance);
}

void to_ros(statistics_msgs::CovarianceStamped& msg, const Eigen::MatrixXd& covariance, const ros::Time& timestamp,
        const std::string& frame_id) {
    to_ros(msg.header, timestamp, frame_id);
    to_ros(msg.covariance, covariance);
}

void to_ros(statistics_msgs::CovarianceFloat32& msg, const Eigen::MatrixXf& covariance) {
    if (covariance.rows() != covariance.cols()) {
        throw std::runtime_error("Cannot convert covariance to ros. Matrix is not square, but " +
                                 std::to_string(covariance.rows()) + " by " + std::to_string(covariance.cols()));
    }
    msg.dimensions = covariance.rows();
    to_ros(msg.matrix, covariance);
}

void to_ros(statistics_msgs::CovarianceFloat32Stamped& msg, const Eigen::MatrixXf& covariance,
        const ros::Time& timestamp, const std::string& frame_id) {
    to_ros(msg.header, timestamp, frame_id);
    to_ros(msg.covariance, covariance);
}

}
