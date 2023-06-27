#ifndef SERPENT_POINTCLOUD_ADD_UNIT_VECTORS_HPP
#define SERPENT_POINTCLOUD_ADD_UNIT_VECTORS_HPP

#include <pcl/PCLPointCloud2.h>
#include <ros/ros.h>

namespace serpent {

class PointcloudAddUnitVectors {
public:
    explicit PointcloudAddUnitVectors(const std::string& input_topic = "input/pointcloud");

    std::string output_topic() const;

private:
    void add_unit_vectors(const pcl::PCLPointCloud2::ConstPtr& msg);

    //// ROS Communication
    ros::NodeHandle nh;
    ros::Publisher pointcloud_publisher;
    ros::Subscriber pointcloud_subscriber;
};

}

#endif
