#include "serpent/front_end/lidar/registration_methods.hpp"

namespace fast_gicp {

RegularizationMethod to_regularization_method(const std::string& method) {
    if (method == "NONE") {
        return RegularizationMethod::NONE;
    } else if (method == "MIN_EIG") {
        return RegularizationMethod::MIN_EIG;
    } else if (method == "NORMALIZED_MIN_EIG") {
        return RegularizationMethod::NORMALIZED_MIN_EIG;
    } else if (method == "PLANE") {
        return RegularizationMethod::PLANE;
    } else if (method == "FROBENIUS") {
        return RegularizationMethod::FROBENIUS;
    }
    throw std::runtime_error("RegularizationMethod \'" + method + "\' not recognized.");
}

VoxelAccumulationMode to_voxel_accumulation_mode(const std::string& mode) {
    if (mode == "ADDITIVE") {
        return VoxelAccumulationMode::ADDITIVE;
    } else if (mode == "ADDITIVE_WEIGHTED") {
        return VoxelAccumulationMode::ADDITIVE_WEIGHTED;
    } else if (mode == "MULTIPLICATIVE") {
        return VoxelAccumulationMode::MULTIPLICATIVE;
    }
    throw std::runtime_error("VoxelAccumulationMode \'" + mode + "\' not recognized.");
}

NeighborSearchMethod to_neighbor_search_method(const std::string& method) {
    if (method == "DIRECT27") {
        return NeighborSearchMethod::DIRECT27;
    } else if (method == "DIRECT7") {
        return NeighborSearchMethod::DIRECT7;
    } else if (method == "DIRECT1") {
        return NeighborSearchMethod::DIRECT1;
    } else if (method == "DIRECT_RADIUS") {  // only CUDA
        return NeighborSearchMethod::DIRECT_RADIUS;
    }
    throw std::runtime_error("NeighborSearchMethod \'" + method + "\' not recognized.");
}

}
