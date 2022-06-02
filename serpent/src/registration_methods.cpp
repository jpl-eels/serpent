#include "serpent/registration_methods.hpp"

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

}
