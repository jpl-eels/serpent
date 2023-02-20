#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pointcloud_tools/point_types.hpp"

pcl::PCLHeader test_header(const unsigned int i = 0);

template<typename PointT>
typename pcl::PointCloud<PointT> test_empty_pointcloud(const unsigned int i = 0);

template<typename PointT>
pcl::PCLPointCloud2 test_empty_pointcloud2(const unsigned int i = 0);

pcl::PointXYZ test_x(const float x = 0.f);

pcl::PointXYZ test_y(const float y = 0.f);

pcl::PointXYZ test_z(const float z = 0.f);

pcl::PointXYZ test_xyz(const float xyz = 0.f);

PointNormalUnit test_pnu_x(const unsigned int i = 0);

PointNormalUnit test_pnu_y(const unsigned int i = 0);

PointNormalUnit test_pnu_z(const unsigned int i = 0);

PointNormalUnit test_pnu_xyz(const unsigned int i = 0);

/* Implementation */

template<typename PointT>
typename pcl::PointCloud<PointT> test_empty_pointcloud(const unsigned int i) {
    typename pcl::PointCloud<PointT> pointcloud;
    pointcloud.header = test_header(i);
    return pointcloud;
}

template<typename PointT>
pcl::PCLPointCloud2 test_empty_pointcloud2(const unsigned int i) {
    typename pcl::PointCloud<PointT> pointcloud = test_empty_pointcloud<PointT>(i);
    pcl::PCLPointCloud2 pointcloud2;
    pcl::toPCLPointCloud2(pointcloud, pointcloud2);
    return pointcloud2;
}
