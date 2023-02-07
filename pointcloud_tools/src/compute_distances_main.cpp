#include <ros/ros.h>

#include "pointcloud_tools/compute_distances.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "compute_distances");
    ros::NodeHandle nh("~");
    const std::string distance_method = nh.param<std::string>("distance_method", "POINT_TO_POINT");
    const std::string source_point_type = nh.param<std::string>("source_point_type", "PointXYZ");
    const std::string target_point_type = nh.param<std::string>("target_point_type", "PointXYZ");
    if (distance_method == "POINT_TO_POINT") {
        if (source_point_type == "PointXYZ" && target_point_type == "PointXYZ") {
            pct::ComputePointToPointDistances<pcl::PointXYZ, pcl::PointXYZ> compute_distances;
            ros::spin();
        } else {
            throw std::runtime_error(
                    "Point types [\"" + source_point_type + "\", \"" + target_point_type + "\"] not yet supported.");
        }
    } else {
        throw std::runtime_error("Distance method \"" + distance_method + "\" not yet supported.");
    }
    return 0;
}
