#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen_ext/geometry.hpp>
#include <sstream>

namespace pct {

void cast_to_float32(pcl::PCLPointCloud2& pointcloud, const std::string& name) {
    pcl::PCLPointField& field = get_field(pointcloud, name);
    if (field.datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32) {
        for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
            if (field.datatype == pcl::PCLPointField::PointFieldTypes::UINT32) {
                const std::uint32_t* t = reinterpret_cast<const std::uint32_t*>(&pointcloud.data[i + field.offset]);
                const float f = static_cast<float>(*t);
                std::memcpy(&pointcloud.data[i + field.offset], &f, sizeof(float));
            } else if (field.datatype == pcl::PCLPointField::PointFieldTypes::INT32) {
                const std::int32_t* t = reinterpret_cast<const std::int32_t*>(&pointcloud.data[i + field.offset]);
                const float f = static_cast<float>(*t);
                std::memcpy(&pointcloud.data[i + field.offset], &f, sizeof(float));
            } else {
                throw std::runtime_error(
                        "Converting field.datatype " + field_type_to_string(field.datatype) + " not supported yet.");
            }
        }
        field.datatype = pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
}

int check_normals(const pcl::PCLPointCloud2& pointcloud, const float threshold) {
    const pcl::PCLPointField& normal_x_field = get_field(pointcloud, "normal_x");
    const pcl::PCLPointField& normal_y_field = get_field(pointcloud, "normal_y");
    const pcl::PCLPointField& normal_z_field = get_field(pointcloud, "normal_z");
    if (normal_x_field.datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32 ||
            normal_y_field.datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32 ||
            normal_z_field.datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32) {
        throw std::runtime_error(
                "Expected normal fields to be FLOAT32 but was " + field_type_to_string(normal_x_field.datatype) + ", " +
                field_type_to_string(normal_y_field.datatype) + ", " + field_type_to_string(normal_z_field.datatype));
    }
    int unnormalised_count{0};
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        const float* normal_x = reinterpret_cast<const float*>(&pointcloud.data[i + normal_x_field.offset]);
        const float* normal_y = reinterpret_cast<const float*>(&pointcloud.data[i + normal_y_field.offset]);
        const float* normal_z = reinterpret_cast<const float*>(&pointcloud.data[i + normal_z_field.offset]);
        const Eigen::Vector3f normal{*normal_x, *normal_y, *normal_z};
        if (std::abs(normal.norm() - 1.f) > threshold) {
            ++unnormalised_count;
            std::cerr << "Normal (" << normal.norm() << "): " << normal[0] << ", " << normal[1] << ", " << normal[2]
                      << "\n";
        }
    }
    return unnormalised_count;
}

void deskew(const Eigen::Isometry3d& skew, const double dt, const pcl::PCLPointCloud2& src, pcl::PCLPointCloud2& dest) {
    // Perform a point cloud copy if there is no transform
    if (skew.isApprox(Eigen::Isometry3d::Identity())) {
        dest = src;
    } else {
        // Error handling
        if (dt <= 0.0) {
            throw std::runtime_error("dt cannot be <= 0.0 for deskewing");
        }

        // Setup
        const Eigen::Isometry3d deskew = skew.inverse();
        const Eigen::Vector3d deskew_translation = deskew.translation();
        const Eigen::Quaterniond deskew_quaternion = Eigen::Quaterniond(deskew.rotation());
        dest.header = src.header;
        dest.height = src.height;
        dest.width = src.width;
        dest.fields = src.fields;
        dest.is_bigendian = src.is_bigendian;
        dest.point_step = src.point_step;
        dest.row_step = src.row_step;
        dest.data.resize(src.data.size());
        dest.is_dense = src.is_dense;
        for (std::uint32_t i = 0; i < src.data.size(); i += src.point_step) {
            Eigen::Vector3d p;
            std::array<pcl::PCLPointField, 3> p_fields;
            double t;
            for (const pcl::PCLPointField& field : src.fields) {
                if (field.name == "x") {
                    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
                        p[0] = static_cast<double>(*reinterpret_cast<const float*>(&src.data[i + field.offset]));
                    } else if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64) {
                        p[0] = *reinterpret_cast<const double*>(&src.data[i + field.offset]);
                    } else {
                        throw std::runtime_error(field.name + " field did not have floating-point datatype");
                    }
                    p_fields[0] = field;
                } else if (field.name == "y") {
                    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
                        p[1] = static_cast<double>(*reinterpret_cast<const float*>(&src.data[i + field.offset]));
                    } else if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64) {
                        p[1] = *reinterpret_cast<const double*>(&src.data[i + field.offset]);
                    } else {
                        throw std::runtime_error(field.name + " field did not have floating-point datatype");
                    }
                    p_fields[1] = field;
                } else if (field.name == "z") {
                    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
                        p[2] = static_cast<double>(*reinterpret_cast<const float*>(&src.data[i + field.offset]));
                    } else if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64) {
                        p[2] = *reinterpret_cast<const double*>(&src.data[i + field.offset]);
                    } else {
                        throw std::runtime_error(field.name + " field did not have floating-point datatype");
                    }
                    p_fields[2] = field;
                } else if (field.name == "t") {
                    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
                        t = static_cast<double>(*reinterpret_cast<const float*>(&src.data[i + field.offset]));
                    } else if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64) {
                        t = *reinterpret_cast<const double*>(&src.data[i + field.offset]);
                    } else {
                        throw std::runtime_error(field.name + " field did not have floating-point datatype");
                    }
                } else {
                    std::memcpy(&dest.data[i + field.offset], &src.data[i + field.offset],
                            pcl::getFieldSize(field.datatype));
                }
            }

            // Compute the deskewed point
            const double interp_fraction = t / dt;
            const Eigen::Vector3d interp_translation = interp_fraction * deskew_translation;
            const Eigen::Quaterniond interp_quaternion =
                    Eigen::Quaterniond::Identity().slerp(interp_fraction, deskew_quaternion);
            const Eigen::Isometry3d interp_transform = eigen_ext::to_transform(interp_translation, interp_quaternion);
            const Eigen::Vector3d p_deskew = interp_transform * p;

            // Copy the deskewed point data
            for (std::size_t j = 0; j < p_fields.size(); ++j) {
                if (p_fields[j].datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
                    const float f = static_cast<float>(p_deskew[j]);
                    std::memcpy(&dest.data[i + p_fields[j].offset], &f, sizeof(float));
                } else if (p_fields[j].datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64) {
                    std::memcpy(&dest.data[i + p_fields[j].offset], &p_deskew[j], sizeof(double));
                } else {
                    throw std::runtime_error("point field did not have floating-point datatype");
                }
            }
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

std::string max_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
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
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(max<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(max<double>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::string min_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch (field.datatype) {
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
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(min<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(min<double>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& time_field) {
    if (time_field.datatype != pcl::traits::asEnum<float>::value) {
        throw std::runtime_error("Currently ns_to_s only handled for FLOAT32");
    }
    for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
        float* t = reinterpret_cast<float*>(&pointcloud.data[i + time_field.offset]);
        *t /= 1.0e9;
    }
}

void ns_to_s(pcl::PCLPointCloud2& pointcloud, const std::string& time_field_name) {
    ns_to_s(pointcloud, get_field(pointcloud, time_field_name));
}

void scale_float32_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const float scale) {
    const pcl::PCLPointField& field = get_field(pointcloud, name);
    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
        for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
            float* f = reinterpret_cast<float*>(&pointcloud.data[i + field.offset]);
            *f *= scale;
        }
    } else {
        throw std::runtime_error("field.datatype was not FLOAT32. field.datatype " +
                                 field_type_to_string(field.datatype) + "Not yet supported.");
    }
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
