#include "pointcloud_tools/pclpointcloud2_utilities.hpp"
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
            } else if (field.datatype ==pcl::PCLPointField::PointFieldTypes::INT32) {
                const std::int32_t* t = reinterpret_cast<const std::int32_t*>(&pointcloud.data[i + field.offset]);
                const float f = static_cast<float>(*t);
                std::memcpy(&pointcloud.data[i + field.offset], &f, sizeof(float));
            } else {
                throw std::runtime_error("Converting field.datatype " + std::to_string(field.datatype)
                        + " not supported yet.");
            }
        }
        field.datatype = pcl::PCLPointField::PointFieldTypes::FLOAT32;
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

void deskew(const Eigen::Isometry3d& transform, const double dt, const pcl::PCLPointCloud2& src,
        pcl::PCLPointCloud2& dest) {
    if (transform.isApprox(Eigen::Isometry3d::Identity())) {
        if (dt <= 0.0) {
            throw std::runtime_error("dt cannot be <= 0.0 for deskewing");
        }
        dest = src;
    } else {
        throw std::runtime_error("Deskew not yet implemented");
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
    ss << "name: " << field.name << ", offset: " << field.offset << ", datatype: " << static_cast<int>(field.datatype)
            << ", count: " << field.count;
    return ss.str();
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
    std::stringstream ss;
    ss << "Pointcloud (" << pointcloud.header.seq << ", " << pointcloud.header.stamp << ", "
            << pointcloud.header.frame_id << ")\n\tsize: h = " << pointcloud.height << ", w = " << pointcloud.width;
    for (const auto& field : pointcloud.fields) {
        ss << "\n\t" << field_string(field) << "\n\t\tmax: " << max_value_str(pointcloud, field) << ", min: "
                << min_value_str(pointcloud, field);
    }
    return ss.str();
}

std::string max_value_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch(field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(max_value<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(max_value<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(max_value<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(max_value<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(max_value<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(max_value<std::uint32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(max_value<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(max_value<double>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

std::string min_value_str(const pcl::PCLPointCloud2& pointcloud, const pcl::PCLPointField& field) {
    switch(field.datatype) {
        case pcl::PCLPointField::PointFieldTypes::INT8:
            return std::to_string(min_value<std::int8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT8:
            return std::to_string(min_value<std::uint8_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT16:
            return std::to_string(min_value<std::int16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT16:
            return std::to_string(min_value<std::uint16_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::INT32:
            return std::to_string(min_value<std::int32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::UINT32:
            return std::to_string(min_value<std::uint32_t>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT32:
            return std::to_string(min_value<float>(pointcloud, field));
        case pcl::PCLPointField::PointFieldTypes::FLOAT64:
            return std::to_string(min_value<double>(pointcloud, field));
        default:
            throw std::runtime_error("Failed to get max value string");
    }
}

void scale_float32_field(pcl::PCLPointCloud2& pointcloud, const std::string& name, const float scale) {
    const pcl::PCLPointField& field = get_field(pointcloud, name);
    if (field.datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) {
        for (std::size_t i = 0; i < pointcloud.data.size(); i += pointcloud.point_step) {
            float* f = reinterpret_cast<float*>(&pointcloud.data[i + field.offset]);
            *f *= scale;
        }
    } else {
        throw std::runtime_error("field.datatype was not FLOAT32. field.datatype " + std::to_string(field.datatype )
                + "Not yet supported.");
    }
}

std::uint32_t size_bytes(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.row_step;
}

std::uint32_t size_points(const pcl::PCLPointCloud2& pointcloud) {
    return pointcloud.height * pointcloud.width;
}

}
