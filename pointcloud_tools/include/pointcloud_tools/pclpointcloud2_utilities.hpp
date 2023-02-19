#ifndef POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_PCLPOINTCLOUD2_UTILITIES_HPP

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <statistics_msgs/SummaryStatistics.h>
#include <statistics_msgs/SummaryStatisticsArray.h>

#include <Eigen/Geometry>
#include <eigen_ext/geometry.hpp>
#include <string>

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

/** Implementation */

template<typename T>
T field_data(const std::uint8_t* field_ptr, const std::uint8_t datatype) {
    return field_data_func<T>(datatype)(field_ptr);
}

template<typename T>
FieldDataFunc<T> field_data_func(const std::uint8_t datatype) {
    switch (datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const float*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const double*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const std::int8_t*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return [](const std::uint8_t* field_ptr) { return static_cast<T>(*field_ptr); };
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const std::int16_t*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const std::uint16_t*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const std::int32_t*>(field_ptr));
            };
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return [](const std::uint8_t* field_ptr) {
                return static_cast<T>(*reinterpret_cast<const std::uint32_t*>(field_ptr));
            };
        default:
            throw std::runtime_error("datatype not recognised");
    }
}

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

template<typename T>
inline bool matches_field_type(const pcl::PCLPointField::PointFieldTypes field_type) {
    return pcl::traits::asEnum<T>::value == field_type;
}

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    if (empty(pointcloud)) {
        throw std::runtime_error("Pointcloud was empty. Max value does not exist.");
    }
    T max_ = std::numeric_limits<T>::lowest();
    auto conversion_func = field_data_func<T>(field.datatype);
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        max_ = std::max(max_, conversion_func(&pointcloud.data[i + field.offset]));
    }
    return max_;
}

template<typename T>
inline T max(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return max<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T = double>
inline T mean(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    const double mean_ = static_cast<double>(sum<T>(pointcloud, field)) / static_cast<double>(size_points(pointcloud));
    return static_cast<T>(mean_);
}

template<typename T = double>
inline T mean(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return mean<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T>
T min(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    if (empty(pointcloud)) {
        throw std::runtime_error("Pointcloud was empty. Min value does not exist.");
    }
    T min_ = std::numeric_limits<T>::max();
    auto conversion_func = field_data_func<T>(field.datatype);
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        min_ = std::min(min_, conversion_func(&pointcloud.data[i + field.offset]));
    }
    return min_;
}

template<typename T>
inline T min(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return min<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> polar_coordinates(const pcl::PCLPointCloud2& pointcloud) {
    const pcl::PCLPointField& x_field = get_field(pointcloud, "x");
    const pcl::PCLPointField& y_field = get_field(pointcloud, "y");
    const pcl::PCLPointField& z_field = get_field(pointcloud, "z");
    auto x_data_func = field_data_func<T>(x_field.datatype);
    auto y_data_func = field_data_func<T>(y_field.datatype);
    auto z_data_func = field_data_func<T>(z_field.datatype);
    Eigen::Matrix<T, 3, Eigen::Dynamic> polar_points;
    polar_points.resize(Eigen::NoChange, size_points(pointcloud));
    for (std::size_t i = 0, j = 0; i < pointcloud.data.size(); i += pointcloud.point_step, ++j) {
        polar_points.col(j) = eigen_ext::cartesian_to_polar<T>(x_data_func(&pointcloud.data[i + x_field.offset]),
                y_data_func(&pointcloud.data[i + y_field.offset]), z_data_func(&pointcloud.data[i + z_field.offset]));
    }
    return polar_points;
}

template<typename T = double>
T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean) {
    return static_cast<T>(std::sqrt(variance<double>(pointcloud, field, mean)));
}

template<typename T = double>
inline T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    return standard_deviation<T>(pointcloud, field, mean<double>(pointcloud, field));
}

template<typename T = double>
inline T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean) {
    return standard_deviation<T>(pointcloud, get_field(pointcloud, field_name), mean);
}

template<typename T = double>
inline T standard_deviation(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return standard_deviation<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T>
T sum(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    T sum_{0};
    auto conversion_func = field_data_func<T>(field.datatype);
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        sum_ += conversion_func(&pointcloud.data[i + field.offset]);
    }
    return sum_;
}

template<typename T>
inline T sum(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return sum<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors(const pcl::PCLPointCloud2& pointcloud) {
    const pcl::PCLPointField& x_field = get_field(pointcloud, "x");
    const pcl::PCLPointField& y_field = get_field(pointcloud, "y");
    const pcl::PCLPointField& z_field = get_field(pointcloud, "z");
    auto x_data_func = field_data_func<T>(x_field.datatype);
    auto y_data_func = field_data_func<T>(y_field.datatype);
    auto z_data_func = field_data_func<T>(z_field.datatype);
    Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors;
    unit_vectors.resize(Eigen::NoChange, size_points(pointcloud));
    for (std::size_t i = 0, j = 0; i < pointcloud.data.size(); i += pointcloud.point_step, ++j) {
        unit_vectors.col(j) = eigen_ext::safe_normalise(x_data_func(&pointcloud.data[i + x_field.offset]),
                y_data_func(&pointcloud.data[i + y_field.offset]), z_data_func(&pointcloud.data[i + z_field.offset]));
    }
    return unit_vectors;
}

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean) {
    double square_residuals_sum_{0.0};
    auto conversion_func = field_data_func<double>(field.datatype);
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        square_residuals_sum_ += std::pow(conversion_func(&pointcloud.data[i + field.offset]) - mean, 2.0);
    }
    square_residuals_sum_ /= static_cast<double>(size_points(pointcloud));
    return static_cast<T>(square_residuals_sum_);
}

template<typename T = double>
inline T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    return variance(pointcloud, field, mean<double>(pointcloud, field));
}

template<typename T = double>
inline T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name, const double mean) {
    return variance<T>(pointcloud, get_field(pointcloud, field_name), mean);
}

template<typename T = double>
inline T variance(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return variance<T>(pointcloud, get_field(pointcloud, field_name));
}

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type);

}

#endif
