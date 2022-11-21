#ifndef EIGEN_EXT_COVARIANCE_HPP
#define EIGEN_EXT_COVARIANCE_HPP

#include <Eigen/Core>

#include "eigen_ext/matrix.hpp"

namespace eigen_ext {

/**
 * @brief Checks the following conditions:
 * - matrix has all finite elements
 * - matrix is square
 * - positive definite
 * - element ij == element ji
 * - elements ii along the diagonal are >= 0
 * 
 * @tparam Derived 
 * @param covariance 
 * @param precision 
 * @return true 
 * @return false 
 */
template<typename Derived>
bool is_valid_covariance(const Eigen::MatrixBase<Derived>& covariance, const typename Derived::Scalar precision = 0);

/**
 * @brief Re-order a covariance matrix by swapping the blocks according to some boundary index.
 * 
 * [  A  |  X  ]    [  B  | X^T ]
 * [-----------] => [-----------]
 * [ X^T |  B  ]    [  X  |  A  ]
 * 
 * @tparam Derived 
 * @param covariance 
 * @param boundary first row/col index of the second block
 * @return Derived 
 */
template<typename Derived>
Derived reorder_covariance(const Eigen::MatrixBase<Derived>& covariance, const Eigen::Index boundary);

/* Implementation */

template<typename Derived>
bool is_valid_covariance(const Eigen::MatrixBase<Derived>& covariance, const typename Derived::Scalar precision) {
    if (!covariance.allFinite() || covariance.rows() != covariance.cols() || is_positive_definite(covariance)) {
        return false;
    }
    for (int r = 0; r < covariance.rows() - 1; ++r) {
        if (covariance(r, r) < static_cast<typename Derived::Scalar>(0)) {
            return false;
        }
        for (int c = r + 1; c < covariance.cols(); ++c) {
            if (std::abs(covariance(r, c) - covariance(c, r)) > precision) {
                return false;
            }
        }
    }
    return true;
}

template<typename Derived>
Derived reorder_covariance(const Eigen::MatrixBase<Derived>& covariance, const Eigen::Index boundary) {
    if (boundary == 0) {
        throw std::runtime_error("Reorder boundary cannot be 0.");
    }
    const Eigen::Index size = covariance.rows();
    if (size != covariance.cols()) {
        throw std::runtime_error("Covariance matrix must be square.");
    }
    if (boundary >= size) {
        throw std::runtime_error("Reorder boundary outside of covariance matrix.");
    }
    Derived reordered_covariance;
    reordered_covariance << covariance.block(boundary, boundary, size - boundary, size - boundary),
            covariance.block(boundary, 0, size - boundary, boundary),
            covariance.block(0, boundary, boundary, size - boundary),
            covariance.block(0, 0, boundary, boundary);
    return reordered_covariance;
}

}

#endif
