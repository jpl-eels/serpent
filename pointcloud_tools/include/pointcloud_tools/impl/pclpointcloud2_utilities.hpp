#ifndef POINTCLOUD_TOOLS_IMPL_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_IMPL_PCLPOINTCLOUD2_UTILITIES_HPP

#include <pcl/point_traits.h>

#include <eigen_ext/geometry.hpp>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

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

}

#endif
