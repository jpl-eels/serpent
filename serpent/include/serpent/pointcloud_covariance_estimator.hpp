#ifndef SERPENT_POINTCLOUD_COVARIANCE_ESTIMATOR_HPP
#define SERPENT_POINTCLOUD_COVARIANCE_ESTIMATOR_HPP

#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>

namespace serpent {

enum PointcloudCovarianceEstimationMethod {
    RANGE,
    RANGE_BIAS
};

PointcloudCovarianceEstimationMethod to_pointcloud_covariance_estimation_method(const std::string& method);

bool is_point_field_method(const PointcloudCovarianceEstimationMethod method);

class PointcloudCovarianceEstimator {
public:
    explicit PointcloudCovarianceEstimator();

private:
    void compute_range_bias_covariance(const pcl::PCLPointCloud2::ConstPtr& msg);

    void compute_range_covariance(const pcl::PCLPointCloud2::ConstPtr& msg);


    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher covariance_publisher;
    ros::Publisher pointcloud_publisher;
    ros::Subscriber pointcloud_subscriber;

    //// Configuration
    PointcloudCovarianceEstimationMethod method;
    float range_noise;
    float range_bias_noise;
};

}

#endif
