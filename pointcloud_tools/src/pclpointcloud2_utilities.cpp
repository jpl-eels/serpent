#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen_ext/geometry.hpp>
#include <sstream>

namespace pct {

pcl::PCLPointCloud2 add_field(const pcl::PCLPointCloud2& src, const std::string& name,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count) {
    return add_fields(src, {name}, datatype, count);
}

pcl::PCLPointCloud2 add_fields(const pcl::PCLPointCloud2& src, const std::vector<std::string>& names,
        const pcl::PCLPointField::PointFieldTypes datatype, const std::uint32_t count) {
    pcl::PCLPointCloud2 dest;
    dest.header = src.header;
    dest.height = src.height;
    dest.width = src.width;
    dest.fields = src.fields;
    for (const auto& name : names) {
        dest.fields.emplace_back(pcl::PCLPointField{.name = name,
                .offset = dest.fields.empty() ? 0 : point_step(dest.fields.back()),
                .datatype = datatype,
                .count = count});
    }
    dest.is_bigendian = src.is_bigendian;
    dest.point_step = point_step(dest.fields.back());
    dest.row_step = row_step(dest);
    dest.data.resize(size_bytes(dest));
    dest.is_dense = src.is_dense;
    for (std::size_t i = 0, j = 0; i < dest.data.size(); i += dest.point_step, j += src.point_step) {
        std::memcpy(&dest.data[i], &src.data[j], src.point_step);
    }
    return dest;
}

pcl::PCLPointCloud2 add_unit_vectors(const pcl::PCLPointCloud2& src) {
    pcl::PCLPointCloud2 dest = add_fields(src, {"ux", "uy", "uz"}, pcl::PCLPointField::PointFieldTypes::FLOAT32, 1);
    auto get_x_field_data = create_get_field_data_function<float>(get_field(dest, "x"));
    auto get_y_field_data = create_get_field_data_function<float>(get_field(dest, "y"));
    auto get_z_field_data = create_get_field_data_function<float>(get_field(dest, "z"));
    auto set_ux_field_data = create_set_field_data_function<float, float>(get_field(dest, "ux"));
    auto set_uy_field_data = create_set_field_data_function<float, float>(get_field(dest, "uy"));
    auto set_uz_field_data = create_set_field_data_function<float, float>(get_field(dest, "uz"));
    const std::size_t num_points = size_points(dest);
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector3f unit_vector = eigen_ext::safe_normalise(get_x_field_data(dest, i),
                get_y_field_data(dest, i), get_z_field_data(dest, i));
        set_ux_field_data(dest, i, unit_vector[0]);
        set_uy_field_data(dest, i, unit_vector[1]);
        set_uz_field_data(dest, i, unit_vector[2]);
    }
    return dest;
}

int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold) {
    auto get_normal_x_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_x"));
    auto get_normal_y_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_y"));
    auto get_normal_z_field_data = create_get_field_data_function<float>(get_field(pointcloud, "normal_z"));
    int unnormalised_count{0};
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector3f normal{get_normal_x_field_data(pointcloud, i), get_normal_y_field_data(pointcloud, i),
                get_normal_z_field_data(pointcloud, i)};
        if (std::abs(normal.norm() - 1.f) > threshold) {
            ++unnormalised_count;
        }
    }
    return unnormalised_count;
}

/**
 * @brief Template specialisation of create_get_field_data_function<InT, T> for when OutT == T == std::uint8_t.
 *
 * It is more efficient as we can directly access the vector data.
 *
 * @tparam
 * @param field
 * @return auto
 */
template<>
auto create_get_field_data_function<std::uint8_t, std::uint8_t>(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<std::uint8_t>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type T");
    }
    const std::size_t offset = field.offset;
    return [offset](const pcl::PCLPointCloud2& pointcloud, const std::size_t i) -> std::uint8_t {
        return pointcloud.data[i * pointcloud.point_step + offset];
    };
}

/**
 * @brief Template specialisation of create_set_field_data_function<InT, T> for when InT == T == std::uint8_t.
 *
 * It is more efficient as we can directly access the vector data.
 *
 * @tparam
 * @param field
 * @return auto
 */
template<>
auto create_set_field_data_function<std::uint8_t, std::uint8_t>(const pcl::PCLPointField& field) {
    if (pcl::traits::asEnum<std::uint8_t>::value != field.datatype) {
        throw std::runtime_error("datatype did not match type std::uint8_t");
    }
    const std::size_t offset = field.offset;
    return [offset](pcl::PCLPointCloud2& pointcloud, const std::size_t i, const std::uint8_t value) -> void {
        pointcloud.data[i * pointcloud.point_step + offset] = value;
    };
}

void deskew(const Eigen::Isometry3d& skew, const double dt, const std::uint64_t new_time,
        const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest) {
    // Skip processing if identity transform.
    dest = src;
    dest.header.stamp = new_time;
    if (!skew.isApprox(Eigen::Isometry3d::Identity())) {
        // Error handling
        if (dt <= 0.0) {
            throw std::runtime_error("dt cannot be <= 0.0 for deskewing");
        }

        // Setup. Skew is T_S^E where S denotes start and E denotes end
        const Eigen::Vector3d skew_translation = skew.translation();
        const Eigen::Quaterniond skew_quaternion = Eigen::Quaterniond(skew.rotation());

        // Compute required quantities
        const double new_time_seconds = static_cast<double>(new_time - src.header.stamp) / 1.0e6;  // us to s
        const double new_time_fraction = new_time_seconds / dt;
        const Eigen::Vector3d new_time_translation = new_time_fraction * skew_translation;
        const Eigen::Quaterniond new_time_quaternion =
                Eigen::Quaterniond::Identity().slerp(new_time_fraction, skew_quaternion);
        // new_time_transform = T_N^S where N denotes new
        const Eigen::Isometry3d new_time_transform =
                eigen_ext::to_transform(new_time_translation, new_time_quaternion).inverse();

        // Access functions
        auto x_field = get_field(dest, "x");
        auto y_field = get_field(dest, "y");
        auto z_field = get_field(dest, "z");
        auto t_field = get_field(dest, "t");
        auto get_x_field_data = create_get_field_data_function<double>(x_field);
        auto get_y_field_data = create_get_field_data_function<double>(y_field);
        auto get_z_field_data = create_get_field_data_function<double>(z_field);
        auto get_t_field_data = create_get_field_data_function<double>(t_field);
        auto set_x_field_data = create_set_field_data_function<double>(x_field);
        auto set_y_field_data = create_set_field_data_function<double>(y_field);
        auto set_z_field_data = create_set_field_data_function<double>(z_field);
        auto set_t_field_data = create_set_field_data_function<float>(t_field);

        const std::size_t num_points = size_points(dest);
        for (std::size_t i = 0; i < num_points; ++i) {
            // Get relevant data from pointcloud
            const Eigen::Vector3d p{get_x_field_data(dest, i), get_y_field_data(dest, i), get_z_field_data(dest, i)};
            const double t = get_t_field_data(dest, i);

            // Compute the deskewed point
            const double interp_fraction = t / dt;
            const Eigen::Vector3d interp_translation = interp_fraction * skew_translation;
            const Eigen::Quaterniond interp_quaternion =
                    Eigen::Quaterniond::Identity().slerp(interp_fraction, skew_quaternion);
            // interp_transform = T_S^i where i is the frame where point i was taken
            const Eigen::Isometry3d interp_transform = eigen_ext::to_transform(interp_translation, interp_quaternion);
            // p_N = T_N^S * T_S^i * p_i
            const Eigen::Vector3d p_deskew = new_time_transform * interp_transform * p;

            // Set the relevant data in the pointcloud
            set_x_field_data(dest, i, p_deskew[0]);
            set_y_field_data(dest, i, p_deskew[1]);
            set_z_field_data(dest, i, p_deskew[2]);
            set_t_field_data(dest, i, 0.0f);
        }
    }
}

void change_field_name(pcl::PCLPointCloud2& pointcloud, const std::string& from, const std::string& to) {
    get_field(pointcloud, from).name = to;
}

bool empty(const pcl::PCLPointCloud2& pointcloud) {
    return size_points(pointcloud) == 0;
}

std::string field_string(const pcl::PCLPointField& field) {
    std::stringstream ss;
    ss << "name: " << field.name << ", offset: " << field.offset
       << ", datatype: " << field_type_to_string(field.datatype) << ", count: " << field.count;
    return ss.str();
}

std::string field_type_to_string(const std::uint8_t field_type) {
    return to_string(static_cast<pcl::PCLPointField::PointFieldTypes>(field_type));
}

const pcl::PCLPointField& get_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (const auto& field : pointcloud.fields) {
        if (field.name == name) {
            return field;
        }
    }
    throw std::runtime_error("Field " + name + " not found in PCLPointCloud2");
}

pcl::PCLPointField& get_field(pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (auto& field : pointcloud.fields) {
        if (field.name == name) {
            return field;
        }
    }
    throw std::runtime_error("Field " + name + " not found in PCLPointCloud2");
}

bool has_field(const pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    for (const auto& field : pointcloud.fields) {
        if (field.name == name) {
            return true;
        }
    }
    return false;
}

std::string info_string(const pcl::PCLPointCloud2& pointcloud) {
    statistics_msgs::SummaryStatisticsArray statistics_ = statistics(pointcloud);
    return info_string(pointcloud, statistics_.statistics);
}

std::string info_string(const pcl::PCLPointCloud2& pointcloud,
        const std::vector<statistics_msgs::SummaryStatistics>& statistics_) {
    if (pointcloud.fields.size() != statistics_.size()) {
        throw std::runtime_error("Fields and statistics had different sizes (" +
                                 std::to_string(pointcloud.fields.size()) + " and " +
                                 std::to_string(statistics_.size()) + ").");
    }
    std::stringstream ss;
    ss << "Pointcloud (" << pointcloud.header.seq << ", " << pointcloud.header.stamp << ", "
       << pointcloud.header.frame_id << ")\n\tsize: h = " << pointcloud.height << ", w = " << pointcloud.width;
    for (std::size_t i = 0; i < pointcloud.fields.size(); ++i) {
        // Even though its inefficient, recompute max and min strings without the cast to double
        ss << "\n\t" << field_string(pointcloud.fields[i]) << "\n\t\tmax: " << max_str(pointcloud, pointcloud.fields[i])
           << ", min: " << min_str(pointcloud, pointcloud.fields[i])
           << ", mean: " << std::to_string(statistics_[i].mean)
           << ", variance: " << std::to_string(statistics_[i].variance);
    }
    return ss.str();
}

bool is_8bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::INT8:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:  // fallthrough
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT16:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_8bit(const std::uint8_t type) {
    return is_8bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_16bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::INT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:  // fallthrough
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:     // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_16bit(const std::uint8_t type) {
    return is_16bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_32bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_32bit(const std::uint8_t type) {
    return is_32bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

bool is_64bit(const pcl::PCLPointField::PointFieldTypes type) {
    switch (type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return true;
        case pcl::PCLPointField::PointFieldTypes::INT8:     // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT8:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT16:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT16:   // fallthrough
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:  // fallthrough
        case pcl::PCLPointField::PointFieldTypes::INT32:    // fallthrough
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return false;
        default:
            throw std::runtime_error("pcl::PCLPointField::PointFieldTypes not recognised.");
    }
}

bool is_64bit(const std::uint8_t type) {
    return is_64bit(static_cast<pcl::PCLPointField::PointFieldTypes>(type));
}

std::string max_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(max<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(max<double>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(max<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(max<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(max<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(max<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(max<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(max<std::uint32_t>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::string min_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(min<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(min<double>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(min<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(min<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(min<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(min<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(min<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(min<std::uint32_t>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::uint32_t point_step(const pcl::PCLPointField& last_field) {
    return last_field.offset + pcl::getFieldSize(last_field.datatype) * last_field.count;
}

void resize(pcl::PCLPointCloud2& pointcloud, const std::uint32_t width, const std::uint32_t height) {
    pointcloud.height = height;
    pointcloud.width = width;
    pointcloud.row_step = row_step(pointcloud);
    pointcloud.data.resize(size_bytes(pointcloud));
}

std::uint32_t row_step(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.point_step * pointcloud.width;
}

void scale_field(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field, const double scale) {
    auto get_field_data = create_get_field_data_function<double>(field);
    auto set_field_data = create_set_field_data_function<double>(field);
    const std::size_t num_points = size_points(pointcloud);
    for (std::size_t i = 0; i < num_points; ++i) {
        set_field_data(pointcloud, i, get_field_data(pointcloud, i) * scale);
    }
}

void scale_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const double scale) {
    return scale_field(pointcloud, get_field(pointcloud, name), scale);
}

// Could also use pointcloud.data.size(), which should be identical if the metadata is correct
std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.row_step;
}

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.width;
}

statistics_msgs::SummaryStatistics statistics(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    statistics_msgs::SummaryStatistics statistics_;
    statistics_.label = field.name;
    statistics_.min = min<double>(pointcloud, field);
    statistics_.max = max<double>(pointcloud, field);
    statistics_.mean = mean<double>(pointcloud, field);
    statistics_.variance = variance<double>(pointcloud, field, statistics_.mean);
    statistics_.count = size_points(pointcloud);
    return statistics_;
}

statistics_msgs::SummaryStatisticsArray statistics(const pcl::PCLPointCloud2& pointcloud) {
    statistics_msgs::SummaryStatisticsArray statistics_array;
    statistics_array.header = pcl_conversions::fromPCL(pointcloud.header);
    for (const pcl::PCLPointField& field : pointcloud.fields) {
        statistics_array.statistics.emplace_back(statistics(pointcloud, field));
    }
    return statistics_array;
}

std::string to_string(const pcl::PCLPointField::PointFieldTypes field_type) {
    switch (field_type) {
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return "FLOAT32";
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return "FLOAT64";
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return "INT8";
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return "INT16";
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return "INT32";
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return "UINT8";
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return "UINT16";
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return "UINT32";
        default:
            throw std::runtime_error("Unknown PointFieldType " + std::to_string(field_type));
    }
}

}

namespace pcl {

bool operator==(const pcl::PCLPointField& f1, const pcl::PCLPointField& f2) {
    return f1.name == f2.name && f1.offset == f2.offset && f1.datatype == f2.datatype && f1.count == f2.count;
}

}
