#ifndef EIGEN_EXT_MATRIX_HPP
#define EIGEN_EXT_MATRIX_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sstream>

namespace eigen_ext {

/**
 * @brief Extract the 3-dim vector of a skew symmetric matrix. Throws an exception if matrix is not skew symmetric.
 *
 * @tparam Scalar
 * @param matrix
 * @return Eigen::Matrix<Scalar, 3, 1>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> from_skew_symmetric(const typename Eigen::Matrix<Scalar, 3, 3>& matrix);

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

/**
 * @brief Check if a matrix is symmetric about its diagonal, according to some level of precision.
 *
 * @tparam Derived
 * @param m
 * @param precision
 * @return true
 * @return false
 */
template<typename Derived>
bool is_symmetric(const Eigen::DenseBase<Derived>& m, const typename Derived::Scalar precision = 0);

/**
 * @brief Returns the (elementwise) linear interpolation of two matrices (or vectors) as:
 *  m1 + interp * (m2 - m1) = (1 - interp) * m1 + interp * m2
 *
 * Normally this function would be used for interpolation with interp in [0, 1] where 0 corresponds to m1 and 1 to m2.
 * However extrapolation is also possible with interp outside [0, 1].
 *
 * @tparam Derived
 * @param m1
 * @param m2
 * @param interp
 * @return Derived
 */
template<typename Derived>
Derived linear_interpolate(const Eigen::DenseBase<Derived>& m1, const Eigen::DenseBase<Derived>& m2,
        const double interp);

/**
 * @brief Create the skew symmetric matrix for a 3-dim vector.
 *
 * @tparam Scalar
 * @param v
 * @return Eigen::Matrix<Scalar, 3, 3>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew_symmetric(const Eigen::Matrix<Scalar, 3, 1>& v);

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

template<typename Derived>
bool is_positive_definite(const typename Eigen::EigenBase<Derived>& m) {
    typename Eigen::LLT<Derived> llt(m);  // compute the Cholesky decomposition
    return llt.info() == Eigen::Success;
}

template<typename Derived>
bool is_symmetric(const Eigen::DenseBase<Derived>& m, const typename Derived::Scalar precision) {
    for (int r = 0; r < m.rows() - 1; ++r) {
        for (int c = r + 1; c < m.cols(); ++c) {
            if (std::abs(m(r, c) - m(c, r)) > precision) {
                return false;
            }
        }
    }
    return true;
}

template<typename Derived>
Derived linear_interpolate(const Eigen::MatrixBase<Derived>& m1, const Eigen::MatrixBase<Derived>& m2,
        const double interp) {
    return (1.0 - interp) * m1.derived() + interp * m2.derived();
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew_symmetric(const Eigen::Matrix<Scalar, 3, 1>& v) {
    Eigen::Matrix<Scalar, 3, 3> m;
    m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
    return m;
}

}

#endif
