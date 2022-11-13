#ifndef EIGEN_EXT_MATRIX_HPP
#define EIGEN_EXT_MATRIX_HPP

#include <Eigen/Core>
#include <sstream>

namespace eigen_ext {

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> from_skew_symmetric(const typename Eigen::Matrix<Scalar, 3, 3>& matrix);

/**
 * @brief Create the skew symmetric matrix for a 3-dim vector.
 *
 * @tparam Scalar
 * @param v
 * @return Eigen::Matrix<Scalar, 3, 3>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew_symmetric(const Eigen::Matrix<Scalar, 3, 1>& v);

/**
 * @brief Check if positive definite through Cholesky decomposition. Note that this function only checks the upper
 * triangular part of the matrix when computing if the matrix is positive definite.
 *
 * @tparam Derived
 * @param m
 * @return true
 * @return false
 */
template<typename Derived>
bool is_positive_definite(const Eigen::EigenBase<Derived>& m);

/* Implementation */

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> from_skew_symmetric(const typename Eigen::Matrix<Scalar, 3, 3>& matrix) {
    if (matrix.diagonal().maxCoeff() > 0.0) {
        std::stringstream ss;
        ss << "Matrix was not skew-symmetric, was:\n" << matrix;
        throw std::runtime_error(ss.str());
    }
    return Eigen::Matrix<Scalar, 3, 1>{matrix(2, 1), matrix(0, 2), matrix(1, 0)};
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew_symmetric(const Eigen::Matrix<Scalar, 3, 1>& v) {
    Eigen::Matrix<Scalar, 3, 3> m;
    m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return m;
}

template<typename Derived>
bool is_positive_definite(const typename Eigen::EigenBase<Derived>& m) {
    typename Eigen::LLT<Derived> llt(m);  // compute the Cholesky decomposition
    return llt.info() == Eigen::Success;
}

}

#endif
