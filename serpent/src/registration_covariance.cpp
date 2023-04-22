#include "serpent/registration_covariance.hpp"

namespace serpent {

bool requires_unit_vectors(const CovarianceEstimationMethod cov_method, const PointCovarianceMethod point_method) {
    bool required_by_cov_method;
    switch (cov_method) {
        case CovarianceEstimationMethod::CONSTANT:  // fallthrough
            required_by_cov_method = false;
            break;
        case CovarianceEstimationMethod::CENSI:  // fallthrough
        case CovarianceEstimationMethod::LLS:
            required_by_cov_method = true;
            break;
        default:
            throw std::runtime_error("CovarianceEstimationMethod not recognised.");
    }
    bool required_by_point_method;
    switch (point_method) {
        case PointCovarianceMethod::CONSTANT:  // fallthrough
        case PointCovarianceMethod::VOXEL_SIZE:
            required_by_point_method = false;
            break;
        case PointCovarianceMethod::RANGE:  // fallthrough
        case PointCovarianceMethod::RANGE_BIAS:
            required_by_point_method = true;
            break;
        default:
            throw std::runtime_error("PointCovarianceMethod not recognised.");
    }
    return required_by_cov_method && required_by_point_method;
}

bool requires_unit_vectors(const std::string& cov_method, const std::string& point_method) {
    return requires_unit_vectors(to_covariance_estimation_method(cov_method), to_point_covariance_method(point_method));
}

CovarianceEstimationMethod to_covariance_estimation_method(const std::string& string) {
    if (string == "CONSTANT") {
        return CovarianceEstimationMethod::CONSTANT;
    } else if (string == "CENSI") {
        return CovarianceEstimationMethod::CENSI;
    } else if (string == "LLS") {
        return CovarianceEstimationMethod::LLS;
    }
    throw std::runtime_error("Could not convert from string \"" + string + "\" to CovarianceEstimationMethod");
}

CovarianceEstimationModel to_covariance_estimation_model(const std::string& string) {
    if (string == "POINT_TO_POINT_LINEARISED") {
        return CovarianceEstimationModel::POINT_TO_POINT_LINEARISED;
    } else if (string == "POINT_TO_POINT_NONLINEAR") {
        return CovarianceEstimationModel::POINT_TO_POINT_NONLINEAR;
    } else if (string == "POINT_TO_PLANE_LINEARISED") {
        return CovarianceEstimationModel::POINT_TO_PLANE_LINEARISED;
    } else if (string == "POINT_TO_PLANE_NONLINEAR") {
        return CovarianceEstimationModel::POINT_TO_PLANE_NONLINEAR;
    }
    throw std::runtime_error("Could not convert from string \"" + string + "\" to CovarianceEstimationModel");
}

PointCovarianceMethod to_point_covariance_method(const std::string& string) {
    if (string == "CONSTANT") {
        return PointCovarianceMethod::CONSTANT;
    } else if (string == "VOXEL_SIZE") {
        return PointCovarianceMethod::VOXEL_SIZE;
    } else if (string == "RANGE") {
        return PointCovarianceMethod::RANGE;
    } else if (string == "RANGE_BIAS") {
        return PointCovarianceMethod::RANGE_BIAS;
    }
    throw std::runtime_error("Could not convert from string \"" + string + "\" to PointCovarianceMethod");
}

std::string to_string(const CovarianceEstimationMethod method) {
    switch (method) {
        case CovarianceEstimationMethod::CONSTANT:
            return "CONSTANT";
        case CovarianceEstimationMethod::CENSI:
            return "CENSI";
        case CovarianceEstimationMethod::LLS:
            return "LLS";
        default:
            throw std::runtime_error("CovarianceEstimationMethod could not be converted to string.");
    }
}

std::string to_string(const CovarianceEstimationModel model) {
    switch (model) {
        case CovarianceEstimationModel::POINT_TO_POINT_LINEARISED:
            return "POINT_TO_POINT_LINEARISED";
        case CovarianceEstimationModel::POINT_TO_POINT_NONLINEAR:
            return "POINT_TO_POINT_NONLINEAR";
        case CovarianceEstimationModel::POINT_TO_PLANE_LINEARISED:
            return "POINT_TO_PLANE_LINEARISED";
        case CovarianceEstimationModel::POINT_TO_PLANE_NONLINEAR:
            return "POINT_TO_PLANE_NONLINEAR";
        default:
            throw std::runtime_error("CovarianceEstimationModel could not be converted to string.");
    }
}

std::string to_string(const PointCovarianceMethod method) {
    switch (method) {
        case PointCovarianceMethod::CONSTANT:
            return "CONSTANT";
        case PointCovarianceMethod::VOXEL_SIZE:
            return "VOXEL_SIZE";
        case PointCovarianceMethod::RANGE:
            return "RANGE";
        case PointCovarianceMethod::RANGE_BIAS:
            return "RANGE_BIAS";
        default:
            throw std::runtime_error("PointCovarianceMethod could not be converted to string.");
    }
}

}
