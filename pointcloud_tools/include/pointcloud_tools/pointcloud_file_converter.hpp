#ifndef POINTCLOUD_TOOLS_POINTCLOUD_FILE_CONVERTER_HPP
#define POINTCLOUD_TOOLS_POINTCLOUD_FILE_CONVERTER_HPP

#include <ros/ros.h>

#include "pointcloud_tools/file_to_message.h"
#include "pointcloud_tools/message_to_file.h"

namespace pct {

class PointcloudFileConverter {
public:
    PointcloudFileConverter();

private:
    bool load(pointcloud_tools::file_to_message::Request& request,
            pointcloud_tools::file_to_message::Response& response);

    bool save(pointcloud_tools::message_to_file::Request& request,
            pointcloud_tools::message_to_file::Response& response);

    // ROS objects
    ros::NodeHandle nh;
    ros::ServiceServer loader;
    ros::ServiceServer saver;
    std::map<std::string, ros::Publisher> publishers;
};

}

#endif
