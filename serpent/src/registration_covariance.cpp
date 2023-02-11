#include "serpent/registration_covariance.hpp"

namespace serpent {

ConstantCovariance::ConstantCovariance(const Eigen::Matrix<double, 6, 6>& constant_covariance)
    : constant_covariance(constant_covariance) {}

ConstantCovariance::ConstantCovariance(const double rotation_noise, const double translation_noise) {
    constant_covariance << Eigen::Matrix<double, 3, 3>::Identity() * rotation_noise * rotation_noise,
            Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Zero(),
            Eigen::Matrix<double, 3, 3>::Identity() * translation_noise * translation_noise;
}

Eigen::Matrix<double, 6, 6> ConstantCovariance::covariance() {
    return constant_covariance;
}

}
