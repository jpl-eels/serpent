#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP

#include <pcl/PCLPointCloud2.h>
#include <statistics_msgs/SummaryStatistics.h>
#include <statistics_msgs/SummaryStatisticsArray.h>

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace pct {

/**
 * @brief Create a new pointcloud with an additional field appended.
 *
 * This function copies all the src data and modifies the pointcloud and field metadata to account for the new field.
 * The bytes corresponding to the new data fields are uninitialised.
 *
 * @param src
 * @param name
 * @param field_type
 * @param count
 * @return pcl::PCLPointCloud2
 */
pcl::PCLPointCloud2 add_field(const pcl::PCLPointCloud2& src, const std::string& name,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count = 1);

pcl::PCLPointCloud2 add_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count = 1);

pcl::PCLPointCloud2 add_unit_vectors(const pcl::PCLPointCloud2& src);

// TODO: generalise this function to other types
void cast_to_float32(pcl::PCLPointCloud2& pointcloud, const std::string& name);

void change_field_name(pcl::PCLPointCloud2& pointcloud, const std::string& from, const std::string& to);

/**
 * @brief Return a count of the number of normals which have a norm different from 1 by a threshold. Therefore it is
 * desirable for this number to be 0. It will always be less than or equal to the number of points in the cloud.
 *
 * @param pointcloud
 * @param threshold
 * @return int
 */
int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold = 0.000001f);

/**
 * @brief Deskew a pointcloud to new_time under the assumption that there has been a constant twist applied over a time
 * dt resulting in a transform from the starting origin frame to the final frame.
 *
 * @param skew skew transform
 * @param dt time of point cloud sweep
 * @param new_time target time to deskew to
 * @param src skewed point cloud
 * @param dest deskewed point cloud
 */
void deskew(const Eigen::Isometry3d& skew, const double dt, const std::uint64_t new_time,
        const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest);

bool empty(const pcl::PCLPointCloud2& pointcloud);

/**
 * @brief Given a pointer to the location of field data, and a field type, return the field data cast to a
 * user-specified type. If one plans to call field_data on many points with the same datatype, use field_data_func.
 *
 * @tparam T
 * @param field_ptr
 * @param field_type
 * @return T
 */
template<typename T>
T field_data(const std::uint8_t* field_ptr, const std::uint8_t datatype);

template<typename T>
using FieldDataFunc = T (*)(const std::uint8_t*);

/**
 * @brief Get a function to convert field data to type T based on a datatype. Use this if you need to get the field data
 * for many points.
 *
 * @tparam T
 * @param datatype
 * @return FieldDataFunc<T>
 */
template<typename T>
FieldDataFunc<T> field_data_func(const std::uint8_t datatype);

std::string field_string(const pcl::PCLPointField& field);

std::string field_type_to_string(const std::uint8_t field_type);

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const pcl::PCLPointField& field,
        const T max);

const pcl::PCLPointField& get_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

pcl::PCLPointField& get_field(pcl::PCLPointCloud2& pointcloud, const std::string& name);

bool has_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

std::string info_string(const pcl::PCLPointCloud2& pointcloud);

std::string info_string(const pcl::PCLPointCloud2& pointcloud,
        const std::vector<statistics_msgs::SummaryStatistics>& statistics);

template<typename T>
bool matches_field_type(const pcl::PCLPointField::PointFieldTypes field_type);

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string max_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T mean(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T mean(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

template<typename T>
T min(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T min(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string min_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& time_field);

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const std::string& time_field_name);

std::uint32_t point_step(const pcl::PCLPointField& last_field);

template<typename T = float>
std::vector<Eigen::Matrix<T, 3, 1>> polar_coordinates(const pcl::PCLPointCloud2& pointcloud);

// TODO: generalise to other types
void scale_float32_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const float scale);

std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud);

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean);

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

statistics_msgs::SummaryStatistics statistics(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

statistics_msgs::SummaryStatisticsArray statistics(const pcl::PCLPointCloud2& pointcloud);

template<typename T>
T sum(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T sum(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type);

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors(const pcl::PCLPointCloud2& pointcloud);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean);

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name);

}

#include "pointcloud_tools/impl/pclpointcloud2_utilities.hpp"

#endif
