#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>

#include <Eigen/Geometry>
#include <string>

namespace pct {

// TODO: generalise this function to other types
void cast_to_float32(pcl::PCLPointCloud2& pointcloud, const std::string& name);

/**
 * @brief Return a count of the number of normals which have a norm different from 1 by a threshold. Therefore it is
 * desirable for this number to be 0. It will always be less than or equal to the number of points in the cloud.
 *
 * @param pointcloud
 * @param threshold
 * @return int
 */
int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold = 0.000001f);

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& time_field);

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const std::string& time_field_name);

/**
 * @brief Deskew a pointcloud under the assumption that there has been a constant twist applied over a time dt resulting
 * in a transform from the starting origin frame to the final frame.
 *
 * @param skew skew transform
 * @param dt time of point cloud sweep
 * @param src skewed point cloud
 * @param dest deskewed point cloud
 */
void deskew(const Eigen::Isometry3d& skew, const double dt, const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest);

void change_field_name(pcl::PCLPointCloud2& pointcloud, const std::string& from, const std::string& to);

bool empty(const pcl::PCLPointCloud2& pointcloud);

std::string field_string(const pcl::PCLPointField& field);

std::string field_type_to_string(const std::uint8_t field_type);

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const pcl::PCLPointField& field,
        const T max) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("Field type did not match user-requested type T");
    }
    dest.header = src.header;
    dest.height = 1;
    dest.width = 0;
    dest.fields = src.fields;
    dest.is_bigendian = src.is_bigendian;
    dest.data.clear();
    dest.data.reserve(src.data.size());
    dest.point_step = src.point_step;
    dest.is_dense = 0;
    for (std::size_t i = 0; i < src.data.size(); i += src.point_step) {
        const T* t = reinterpret_cast<const T*>(&src.data[i + field.offset]);
        if (*t <= max) {
            // Copy the point data
            dest.data.resize(dest.data.size() + dest.point_step);
            std::memcpy(&dest.data[dest.data.size() - dest.point_step], &src.data[i], dest.point_step);
            dest.width += 1;
        }
    }
    dest.row_step = dest.point_step * dest.width;
}

const pcl::PCLPointField& get_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

pcl::PCLPointField& get_field(pcl::PCLPointCloud2& pointcloud, const std::string& name);

bool has_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name);

std::string info_string(const pcl::PCLPointCloud2& pointcloud);

template<typename T>
T max_value(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("Field type did not match user-requested type T");
    }
    if (empty(pointcloud)) {
        throw std::runtime_error("Pointcloud was empty");
    }
    T max = std::numeric_limits<T>::lowest();
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        const T* t = reinterpret_cast<const T*>(&pointcloud.data[i + field.offset]);
        max = std::max(max, *t);
    }
    return max;
}

template<typename T>
inline T max_value(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return max_value<T>(pointcloud, get_field(pointcloud, field_name));
}

std::string max_value_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

template<typename T>
T min_value(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("Field type did not match user-requested type T");
    }
    if (empty(pointcloud)) {
        throw std::runtime_error("Pointcloud was empty");
    }
    T min = std::numeric_limits<T>::max();
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        const T* t = reinterpret_cast<const T*>(&pointcloud.data[i + field.offset]);
        min = std::min(min, *t);
    }
    return min;
}

template<typename T>
inline T min_value(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return min_value<T>(pointcloud, get_field(pointcloud, field_name));
}

std::string min_value_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field);

// TODO: generalise to other types
void scale_float32_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const float scale);

std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud);

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud);

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type);

}

#endif
