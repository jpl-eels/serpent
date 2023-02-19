#include "serpent/pointcloud_add_unit_vectors.hpp"

#include <pcl_ros/point_cloud.h>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace serpent {

PointcloudAddUnitVectors::PointcloudAddUnitVectors(const std::string& input_topic)
    : nh("serpent") {
    // Publisher and subscribers
    pointcloud_subscriber =
            nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, &PointcloudAddUnitVectors::add_unit_vectors, this);
    pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>("add_unit_vectors/pointcloud", 1);
}

std::string PointcloudAddUnitVectors::output_topic() const {
    return pointcloud_publisher.getTopic();
}

void PointcloudAddUnitVectors::add_unit_vectors(const pcl::PCLPointCloud2::ConstPtr& msg) {
    const ros::WallTime tic = ros::WallTime::now();

    pointcloud_publisher.publish(pct::add_unit_vectors(*msg));

    ROS_INFO_STREAM("Pointcloud unit vectors computed in " << (ros::WallTime::now() - tic).toSec() << " s.");
}

}
