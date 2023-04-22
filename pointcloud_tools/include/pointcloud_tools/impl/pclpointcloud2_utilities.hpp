#ifndef POINTCLOUD_TOOLS_IMPL_PCLPOINTCLOUD2_UTILITIES_HPP
#define POINTCLOUD_TOOLS_IMPL_PCLPOINTCLOUD2_UTILITIES_HPP

#include <eigen_ext/geometry.hpp>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

/**
 * @brief Implementation of create_get_field_data_function<OutT, T> for when OutT != T. Casting is required.
 *
 * Implementation note: *reinterpret_cast<const T*>(&data[...]) is undefined behaviour due to strict aliasing. Thus a
 * memcpy is required.
 *
 * @tparam OutT
 * @tparam T
 * @param field
 * @return auto
 */
template<typename OutT, typename T, std::enable_if_t<!std::is_same_v<OutT, T>, int> = 0>
auto create_get_field_data_function(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](const pcl::PCLPointCloud2& pointcloud, const std::size_t i) -> OutT {
        T t;
        std::memcpy(&t, &pointcloud.data[i * pointcloud.point_step + offset], sizeof(T));
        return static_cast<OutT>(t);
    };
}

/**
 * @brief Implementation of create_set_field_data_function<OutT, T> for when OutT == T.
 *
 * Implementation note: *reinterpret_cast<const T*>(&data[...]) is undefined behaviour due to strict aliasing. Thus a
 * memcpy is required.
 *
 * @tparam OutT
 * @tparam T
 * @param field
 * @return auto
 */
template<typename OutT, typename T, std::enable_if_t<std::is_same_v<OutT, T>, int> = 0>
auto create_get_field_data_function(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](const pcl::PCLPointCloud2& pointcloud, const std::size_t i) -> OutT {
        T t;
        std::memcpy(&t, &pointcloud.data[i * pointcloud.point_step + offset], sizeof(T));
        return t;
    };
}

/**
 * @brief Define a helper function prototype for setting field data, required by create_get_field_data_function<OutT>
 *
 * @tparam OutT
 */
template<typename OutT>
using GetFieldDataFunctionHelper = OutT (*)(const pcl::PCLPointCloud2&, const std::size_t, const std::size_t);

/**
 * @brief Create a helper function reqiured by create_get_field_data_function<OutT>, for when OutT == T.
 *
 * This function ought not to be used directly.
 *
 * @tparam OutT
 * @tparam T
 * @param field
 * @return GetFieldDataFunctionHelper<OutT>
 */
template<typename OutT, typename T, std::enable_if_t<std::is_same_v<OutT, T>, int> = 0>
GetFieldDataFunctionHelper<OutT> create_get_field_data_function_helper(const pcl::PCLPointField& field) {
    return [](const pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset) -> OutT {
        T t;
        std::memcpy(&t, &pointcloud.data[i * pointcloud.point_step + offset], sizeof(T));
        return t;
    };
}

/**
 * @brief Template specialisation of create_get_field_data_function_helper.
 *
 * @tparam
 * @param field
 * @return GetFieldDataFunctionHelper<std::uint8_t>
 */
template<>
inline GetFieldDataFunctionHelper<std::uint8_t> create_get_field_data_function_helper<std::uint8_t, std::uint8_t>(
        const pcl::PCLPointField& field) {
    return [](const pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset) -> std::uint8_t {
        return pointcloud.data[i * pointcloud.point_step + offset];
    };
}

/**
 * @brief Create a helper function reqiured by create_get_field_data_function<OutT>, for when OutT != T.
 *
 * This function ought not to be used directly.
 *
 * @tparam OutT
 * @tparam T
 * @param field
 * @return GetFieldDataFunctionHelper<OutT>
 */
template<typename OutT, typename T, std::enable_if_t<!std::is_same_v<OutT, T>, int> = 0>
GetFieldDataFunctionHelper<OutT> create_get_field_data_function_helper(const pcl::PCLPointField& field) {
    return [](const pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset) -> OutT {
        T t;
        std::memcpy(&t, &pointcloud.data[i * pointcloud.point_step + offset], sizeof(T));
        return static_cast<OutT>(t);
    };
}

/**
 * @brief A helper function is required. This is because each lambda function has a unique type and so we cannot return
 * a lambda in each of the different type cases (even if they had the same function signature) and as before, since we
 * are capturing the offset, we cannot return a function pointer.
 *
 * @tparam OutT
 * @param field
 * @return auto
 */
template<typename OutT>
auto create_get_field_data_function(const pcl::PCLPointField& field) {
    GetFieldDataFunctionHelper<OutT> helper;
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT64>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT8:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT8>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT8>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT16:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT16>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT16>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT32:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            helper = create_get_field_data_function_helper<OutT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT32>::type>(field);
            break;
        default:
            throw std::runtime_error("field datatype not recognised");
    }
    const std::size_t offset = field.offset;
    return [helper, offset](const pcl::PCLPointCloud2& pointcloud, const std::size_t i) -> OutT {
        return helper(pointcloud, i, offset);
    };
}

/**
 * @brief Implementation of create_set_field_data_function<InT, T> for when InT != T. Casting is required.
 *
 * Implementation note: *reinterpret_cast<const T*>(&data[...]) is undefined behaviour due to strict aliasing. Thus a
 * memcpy is required.
 *
 * @tparam InT
 * @tparam T
 * @param field
 * @return auto
 */
template<typename InT, typename T, std::enable_if_t<!std::is_same_v<InT, T>, int> = 0>
auto create_set_field_data_function(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const InT value_in) -> void {
        const T value = static_cast<T>(value_in);
        std::memcpy(&pointcloud.data[i * pointcloud.point_step + offset], &value, sizeof(T));
    };
}

/**
 * @brief Implementation of create_set_field_data_function<InT, T> for when InT == T.
 *
 * Implementation note: *reinterpret_cast<const T*>(&data[...]) is undefined behaviour due to strict aliasing. Thus a
 * memcpy is required.
 *
 * Note specialisation for InT == T == std::uint8_t in source file.
 *
 * @tparam InT
 * @tparam T
 * @param field
 * @return auto
 */
template<typename InT, typename T, std::enable_if_t<std::is_same_v<InT, T>, int> = 0>
auto create_set_field_data_function(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<T>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const InT value) -> void {
        std::memcpy(&pointcloud.data[i * pointcloud.point_step + offset], &value, sizeof(T));
    };
}

/**
 * @brief Define a helper function prototype for setting field data, required by create_set_field_data_function<InT>
 *
 * @tparam InT
 */
template<typename InT>
using SetFieldDataFunctionHelper = void (*)(pcl::PCLPointCloud2&, const std::size_t, const std::size_t, const InT);

/**
 * @brief Create a helper function reqiured by create_set_field_data_function<InT>, for when InT == T.
 *
 * This function ought not to be used directly.
 *
 * @tparam InT
 * @tparam T
 * @param field
 * @return SetFieldDataFunctionHelper<InT>
 */
template<typename InT, typename T, std::enable_if_t<std::is_same_v<InT, T>, int> = 0>
SetFieldDataFunctionHelper<InT> create_set_field_data_function_helper(const pcl::PCLPointField& field) {
    return [](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset, const InT value) -> void {
        std::memcpy(&pointcloud.data[i * pointcloud.point_step + offset], &value, sizeof(T));
    };
}

/**
 * @brief Template specialisation of create_set_field_data_function_helper.
 *
 * @tparam
 * @param field
 * @return SetFieldDataFunctionHelper<std::uint8_t>
 */
template<>
inline SetFieldDataFunctionHelper<std::uint8_t> create_set_field_data_function_helper<std::uint8_t, std::uint8_t>(
        const pcl::PCLPointField& field) {
    return [](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset,
                   const std::uint8_t value) -> void { pointcloud.data[i * pointcloud.point_step + offset] = value; };
}

/**
 * @brief Create a helper function reqiured by create_set_field_data_function<InT>, for when InT != T.
 *
 * This function ought not to be used directly.
 *
 * @tparam InT
 * @tparam T
 * @param field
 * @return SetFieldDataFunctionHelper<InT>
 */
template<typename InT, typename T, std::enable_if_t<!std::is_same_v<InT, T>, int> = 0>
SetFieldDataFunctionHelper<InT> create_set_field_data_function_helper(const pcl::PCLPointField& field) {
    return [](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::size_t offset,
                   const InT value_in) -> void {
        const T value = static_cast<T>(value_in);
        std::memcpy(&pointcloud.data[i * pointcloud.point_step + offset], &value, sizeof(T));
    };
}

/**
 * @brief A helper function is required. This is because each lambda function has a unique type and so we cannot return
 * a lambda in each of the different type cases (even if they had the same function signature) and as before, since we
 * are capturing the offset, we cannot return a function pointer.
 *
 * @tparam InT
 * @param field
 * @return auto
 */
template<typename InT>
auto create_set_field_data_function(const pcl::PCLPointField& field) {
    SetFieldDataFunctionHelper<InT> helper;
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT64>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT8:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT8>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT8>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT16:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT16>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT16>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::INT32:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(field);
            break;
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            helper = create_set_field_data_function_helper<InT,
                    pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::UINT32>::type>(field);
            break;
        default:
            throw std::runtime_error("field datatype not recognised");
    }
    const std::size_t offset = field.offset;
    return [helper, offset](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const InT value) -> void {
        return helper(pointcloud, i, offset, value);
    };
}

template<std::uint8_t type>
void cast_field(pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    using CastType = typename pcl::traits::asType<type>::type;

    // Check bit-depths match
    pcl::PCLPointField& field = get_field(pointcloud, name);
    if (!is_same_bit_depth<type>(field.datatype)) {
        throw std::runtime_error("Casting between different bit depths is not yet supported.");
    }

    // Only cast if the type actually changes
    if (field.datatype != type) {
        auto get_field_data = create_get_field_data_function<CastType>(field);
        field.datatype = type;
        auto set_field_data = create_set_field_data_function<CastType, CastType>(field);
        const std::size_t num_points = size_points(pointcloud);
        for (std::size_t i = 0; i < num_points; ++i) {
            set_field_data(pointcloud, i, get_field_data(pointcloud, i));
        }
    }
}

template<std::uint8_t type>
void cast_field_with_scale(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale) {
    using CastType = typename pcl::traits::asType<type>::type;

    // Check bit-depths match
    pcl::PCLPointField& field = get_field(pointcloud, name);
    if (!is_same_bit_depth<type>(field.datatype)) {
        throw std::runtime_error("Casting between different bit depths is not yet supported.");
    }

    // Only cast if the type actually changes
    if (field.datatype != type) {
        auto get_field_data = create_get_field_data_function<double>(field);
        field.datatype = type;
        auto set_field_data = create_set_field_data_function<double, CastType>(field);
        const std::size_t num_points = size_points(pointcloud);
        for (std::size_t i = 0; i < num_points; ++i) {
            set_field_data(pointcloud, i, get_field_data(pointcloud, i) * scale);
        }
    }
}

template<typename OutT>
OutT field_data(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const std::size_t i) {
    return create_get_field_data_function<OutT>(field)(pointcloud, i);
}

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const pcl::PCLPointField& field,
        const T max) {
    // Setup
    dest.header = src.header;
    dest.height = 1;
    dest.width = 0;
    dest.fields = src.fields;
    dest.is_bigendian = src.is_bigendian;
    dest.point_step = src.point_step;
    dest.data.clear();
    // Assume all bytes will be copied across initially
    dest.data.resize(size_bytes(src));
    dest.is_dense = 0;

    auto get_field_data = create_get_field_data_function<T>(field);
    const std::size_t num_src_points = size_points(src);
    for (std::size_t i = 0; i < num_src_points; ++i) {
        if (get_field_data(src, i) <= max) {
            std::memcpy(&dest.data[dest.width * dest.point_step], &src.data[i * src.point_step], dest.point_step);
            ++dest.width;
        }
    }

    // Compute row_step and resize the data to remove excess
    dest.row_step = row_step(dest);
    dest.data.resize(size_bytes(dest));
}

template<typename T>
void filter_max(const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest, const std::string& field_name, const T max) {
    filter_max<T>(src, dest, get_field(src, field_name), max);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::INT8>(const std::uint8_t runtime_type) {
    return is_8bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::UINT8>(const std::uint8_t runtime_type) {
    return is_8bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::INT16>(const std::uint8_t runtime_type) {
    return is_16bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::UINT16>(const std::uint8_t runtime_type) {
    return is_16bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::INT32>(const std::uint8_t runtime_type) {
    return is_32bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::UINT32>(const std::uint8_t runtime_type) {
    return is_32bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::FLOAT32>(const std::uint8_t runtime_type) {
    return is_32bit(runtime_type);
}

template<>
inline bool is_same_bit_depth<pcl::PCLPointField::FLOAT64>(const std::uint8_t runtime_type) {
    return is_64bit(runtime_type);
}

template<>
struct is_8bit_type<pcl::PCLPointField::INT8> {
    static const bool value = true;
};
template<>
struct is_8bit_type<pcl::PCLPointField::UINT8> {
    static const bool value = true;
};
template<>
struct is_8bit_type<pcl::PCLPointField::INT16> {
    static const bool value = false;
};
template<>
struct is_8bit_type<pcl::PCLPointField::UINT16> {
    static const bool value = false;
};
template<>
struct is_8bit_type<pcl::PCLPointField::INT32> {
    static const bool value = false;
};
template<>
struct is_8bit_type<pcl::PCLPointField::UINT32> {
    static const bool value = false;
};
template<>
struct is_8bit_type<pcl::PCLPointField::FLOAT32> {
    static const bool value = false;
};
template<>
struct is_8bit_type<pcl::PCLPointField::FLOAT64> {
    static const bool value = false;
};

template<>
struct is_16bit_type<pcl::PCLPointField::INT8> {
    static const bool value = false;
};
template<>
struct is_16bit_type<pcl::PCLPointField::UINT8> {
    static const bool value = false;
};
template<>
struct is_16bit_type<pcl::PCLPointField::INT16> {
    static const bool value = true;
};
template<>
struct is_16bit_type<pcl::PCLPointField::UINT16> {
    static const bool value = true;
};
template<>
struct is_16bit_type<pcl::PCLPointField::INT32> {
    static const bool value = false;
};
template<>
struct is_16bit_type<pcl::PCLPointField::UINT32> {
    static const bool value = false;
};
template<>
struct is_16bit_type<pcl::PCLPointField::FLOAT32> {
    static const bool value = false;
};
template<>
struct is_16bit_type<pcl::PCLPointField::FLOAT64> {
    static const bool value = false;
};

template<>
struct is_32bit_type<pcl::PCLPointField::INT8> {
    static const bool value = false;
};
template<>
struct is_32bit_type<pcl::PCLPointField::UINT8> {
    static const bool value = false;
};
template<>
struct is_32bit_type<pcl::PCLPointField::INT16> {
    static const bool value = false;
};
template<>
struct is_32bit_type<pcl::PCLPointField::UINT16> {
    static const bool value = false;
};
template<>
struct is_32bit_type<pcl::PCLPointField::INT32> {
    static const bool value = true;
};
template<>
struct is_32bit_type<pcl::PCLPointField::UINT32> {
    static const bool value = true;
};
template<>
struct is_32bit_type<pcl::PCLPointField::FLOAT32> {
    static const bool value = true;
};
template<>
struct is_32bit_type<pcl::PCLPointField::FLOAT64> {
    static const bool value = false;
};

template<>
struct is_64bit_type<pcl::PCLPointField::INT8> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::UINT8> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::INT16> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::UINT16> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::INT32> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::UINT32> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::FLOAT32> {
    static const bool value = false;
};
template<>
struct is_64bit_type<pcl::PCLPointField::FLOAT64> {
    static const bool value = true;
};

template<typename T>
T max(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    if (empty(pointcloud)) {
        throw std::runtime_error("Pointcloud was empty. Max value does not exist.");
    }
    T max_ = std::numeric_limits<T>::lowest();
    auto get_field_data = create_get_field_data_function<T>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        max_ = std::max(max_, get_field_data(pointcloud, i));
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
    auto get_field_data = create_get_field_data_function<T>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        min_ = std::min(min_, get_field_data(pointcloud, i));
    }
    return min_;
}

template<typename T>
inline T min(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return min<T>(pointcloud, get_field(pointcloud, field_name));
}

template<pcl::PCLPointField::PointFieldTypes type>
void scale_and_cast_field(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double scale) {
    auto get_field_data = create_get_field_data_function<double>(field);
    auto set_field_data = create_set_field_data_function<double>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        set_field_data(pointcloud, i, get_field_data(pointcloud, i) * scale);
    }
}

template<pcl::PCLPointField::PointFieldTypes type>
void scale_and_cast_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale) {
    return scale_and_cast_field<type>(pointcloud, name, scale);
}

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> polar_coordinates(const pcl::PCLPointCloud2& pointcloud) {
    auto get_x_field_data = create_get_field_data_function<T>(get_field(pointcloud, "x"));
    auto get_y_field_data = create_get_field_data_function<T>(get_field(pointcloud, "y"));
    auto get_z_field_data = create_get_field_data_function<T>(get_field(pointcloud, "z"));
    Eigen::Matrix<T, 3, Eigen::Dynamic> polar_points;
    const std::size_t num_points = size_points(pointcloud);
    polar_points.resize(Eigen::NoChange, num_points);
    for (std::size_t i = 0; i < num_points; ++i) {
        polar_points.col(i) = eigen_ext::cartesian_to_polar<T>(get_x_field_data(pointcloud, i),
                get_y_field_data(pointcloud, i), get_z_field_data(pointcloud, i));
    }
    return polar_points;
}

template<typename InT>
void set_field_data(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const std::size_t i,
        const InT value) {
    return create_set_field_data_function<InT>(field)(pointcloud, i, value);
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
    auto get_field_data = create_get_field_data_function<T>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        sum_ += get_field_data(pointcloud, i);
    }
    return sum_;
}

template<typename T>
inline T sum(const pcl::PCLPointCloud2& pointcloud, const std::string& field_name) {
    return sum<T>(pointcloud, get_field(pointcloud, field_name));
}

template<typename T = float>
Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors(const pcl::PCLPointCloud2& pointcloud) {
    auto convert_x_field_data = create_get_field_data_function<T>(get_field(pointcloud, "x"));
    auto convert_y_field_data = create_get_field_data_function<T>(get_field(pointcloud, "y"));
    auto convert_z_field_data = create_get_field_data_function<T>(get_field(pointcloud, "z"));
    Eigen::Matrix<T, 3, Eigen::Dynamic> unit_vectors;
    const std::size_t num_points = size_points(pointcloud);
    unit_vectors.resize(Eigen::NoChange, num_points);
    for (std::size_t i = 0; i < num_points; ++i) {
        unit_vectors.col(i) = eigen_ext::safe_normalise(convert_x_field_data(pointcloud, i),
                convert_y_field_data(pointcloud, i), convert_z_field_data(pointcloud, i));
    }
    return unit_vectors;
}

template<typename T = double>
T variance(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double mean) {
    double square_residuals_sum_{0.0};
    auto get_field_data = create_get_field_data_function<double>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        square_residuals_sum_ += std::pow(get_field_data(pointcloud, i) - mean, 2.0);
    }
    square_residuals_sum_ /= static_cast<double>(num_points);
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
