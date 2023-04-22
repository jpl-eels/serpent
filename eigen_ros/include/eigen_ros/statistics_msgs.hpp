#include <ros/ros.h>
#include <statistics_msgs/Covariance.h>
#include <statistics_msgs/CovarianceFloat32.h>
#include <statistics_msgs/CovarianceFloat32Stamped.h>
#include <statistics_msgs/CovarianceStamped.h>

#include <Eigen/Core>
#include <string>

namespace eigen_ros {

void from_ros(const statistics_msgs::Covariance& msg, Eigen::MatrixXd& covariance);

void from_ros(const statistics_msgs::CovarianceStamped& msg, Eigen::MatrixXd& covariance, ros::Time& timestamp,
        std::string& frame_id);

void from_ros(const statistics_msgs::CovarianceFloat32& msg, Eigen::MatrixXf& covariance);

void from_ros(const statistics_msgs::CovarianceFloat32Stamped& msg, Eigen::MatrixXf& covariance, ros::Time& timestamp,
        std::string& frame_id);

void to_ros(statistics_msgs::Covariance& msg, const Eigen::MatrixXd& covariance);

void to_ros(statistics_msgs::CovarianceStamped& msg, const Eigen::MatrixXd& covariance, const ros::Time& timestamp,
        const std::string& frame_id);

void to_ros(statistics_msgs::CovarianceFloat32& msg, const Eigen::MatrixXf& covariance);

void to_ros(statistics_msgs::CovarianceFloat32Stamped& msg, const Eigen::MatrixXf& covariance,
        const ros::Time& timestamp, const std::string& frame_id);

}
