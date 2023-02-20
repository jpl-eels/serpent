#ifndef SERPENT_IMPL_UTILITIES_HPP
#define SERPENT_IMPL_UTILITIES_HPP

#include "serpent/utilities.hpp"

namespace serpent {

template<typename Scalar>
Scalar interpolate(const Scalar value1, const Scalar value2, const ros::Time& time1, const ros::Time& time2,
        const ros::Time& interpolation_time) {
    static_assert(std::is_floating_point<Scalar>::value, "Scalar is not a floating point type");
    if (time2 <= time1 || interpolation_time < time1 || interpolation_time > time2) {
        throw std::runtime_error("Interpolation requires time1 < time2 and time1 <= interpolation_time <= time2.");
    }
    const double interp = (interpolation_time - time1).toSec() / (time2 - time1).toSec();
    return (1.0 - interp) * value1 + interp * value2;
}

template<typename Scalar, int Size>
std::string to_flat_string(const typename Eigen::Matrix<Scalar, Size, 1>& vector) {
    std::stringstream ss;
    for (std::size_t i = 0; i < vector.rows(); ++i) {
        ss << (i > 0 ? " " : "") << vector[i];
    }
    return ss.str();
}

}

#endif
