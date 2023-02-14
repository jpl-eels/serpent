#ifndef SERPENT_POINTCLOUD_FILTER_HPP
#define SERPENT_POINTCLOUD_FILTER_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

namespace serpent {

class PointcloudFilter {
public:
    explicit PointcloudFilter();

private:
    void body_filter_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    void random_sample_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    void range_filter_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    void statistical_outlier_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    void voxel_grid_callback(const pcl::PCLPointCloud2::ConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Publisher body_pointcloud_publisher;
    ros::Publisher random_sample_pointcloud_publisher;
    ros::Publisher range_pointcloud_publisher;
    ros::Publisher statistical_outlier_pointcloud_publisher;
    ros::Publisher voxel_grid_pointcloud_publisher;
    ros::Subscriber body_pointcloud_subscriber;
    ros::Subscriber random_sample_pointcloud_subscriber;
    ros::Subscriber range_pointcloud_subscriber;
    ros::Subscriber statistical_outlier_pointcloud_subscriber;
    ros::Subscriber voxel_grid_pointcloud_subscriber;

    //// Filters
    pcl::CropBox<pcl::PCLPointCloud2> body_filter;
    pcl::PassThrough<pcl::PCLPointCloud2> range_filter;
    pcl::RandomSample<pcl::PCLPointCloud2> random_sample_filter;
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor_filter;
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid_filter;
};

}

#endif
