#include "pointcloud_tools/pointcloud_file_converter.hpp"

#include <pcl/io/auto_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

PointcloudFileConverter::PointcloudFileConverter()
    : nh("~") {
    loader = nh.advertiseService("load_pointcloud", &PointcloudFileConverter::load, this);
    saver = nh.advertiseService("save_pointcloud", &PointcloudFileConverter::save, this);
}

bool PointcloudFileConverter::load(pointcloud_tools::file_to_message::Request& request,
        pointcloud_tools::file_to_message::Response& response) {
    auto it = publishers.find(request.topic);
    if (it == publishers.end()) {
        std::pair<std::map<std::string, ros::Publisher>::iterator, bool> emplace_it;
        if (request.as_pcl_type) {
            emplace_it = publishers.emplace(request.topic,
                    nh.advertise<pcl::PCLPointCloud2>(request.topic, 1, request.latch));
        } else {
            emplace_it = publishers.emplace(request.topic,
                    nh.advertise<sensor_msgs::PointCloud2>(request.topic, 1, request.latch));
        }
        if (!emplace_it.second) {
            throw std::runtime_error("Failed to create publisher for topic \'" + request.topic + "'\'");
        }
        it = emplace_it.first;
    }
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    if (pcl::io::load(request.filepath, *pointcloud) != 0) {
        throw std::runtime_error("Failed to read pointcloud from file \'" + request.filepath + "\'");
    }
    pcl_conversions::toPCL(ros::Time::now(), pointcloud->header.stamp);
    pointcloud->header.frame_id = request.frame_id;
    if (request.as_pcl_type) {
        it->second.publish(pointcloud);
    } else {
        sensor_msgs::PointCloud2 msg;
        pcl_conversions::fromPCL(*pointcloud, msg);
        it->second.publish(msg);
    }
    response.result_msg = "Published pointcloud (" + std::to_string(pointcloud->height * pointcloud->width) +
                          " points) to topic \'" + request.topic + "\'";
    return true;
}

bool PointcloudFileConverter::save(pointcloud_tools::message_to_file::Request& request,
        pointcloud_tools::message_to_file::Response& response) {
    // These must be kept in scope, one is used for each message type
    boost::shared_ptr<pcl::PCLPointCloud2 const> msg_pcl_ptr;
    pcl::PCLPointCloud2 msg_pcl;
    // Pointcloud pointer for saving
    const pcl::PCLPointCloud2* pointcloud;
    if (request.is_pcl_type) {
        msg_pcl_ptr =
                ros::topic::waitForMessage<pcl::PCLPointCloud2>(request.topic, nh, ros::Duration(request.timeout));
        if (!msg_pcl_ptr) {
            throw std::runtime_error("Failed to receive PCL pointcloud on topic \'" + request.topic +
                                     "\' within timeout period of " + std::to_string(request.timeout) + " seconds");
        }
        pointcloud = msg_pcl_ptr.get();
    } else {
        const boost::shared_ptr<sensor_msgs::PointCloud2 const> msg =
                ros::topic::waitForMessage<sensor_msgs::PointCloud2>(request.topic, nh, ros::Duration(request.timeout));
        if (!msg) {
            throw std::runtime_error("Failed to receive pointcloud on topic \'" + request.topic +
                                     "\' within timeout period of " + std::to_string(request.timeout) + " seconds");
        }
        pcl_conversions::toPCL(*msg, msg_pcl);
        pointcloud = &msg_pcl;
    }
    if (pcl::io::save(request.filepath, *pointcloud) != 0) {
        throw std::runtime_error("Failed to save pointcloud to " + request.filepath);
    }
    response.result_msg = "Saved PCL pointcloud (" + std::to_string(pointcloud->width * pointcloud->height) +
                          " points) to " + request.filepath;
    return true;
}
