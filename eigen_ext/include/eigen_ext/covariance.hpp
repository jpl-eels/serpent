#ifndef EIGEN_EXT_COVARIANCE_HPP
#define EIGEN_EXT_COVARIANCE_HPP

#include <Eigen/Core>

namespace eigen_ext {

template<typename Derived>
bool is_valid_covariance(const Eigen::MatrixBase<Derived>& covariance, const typename Derived::Scalar precision);

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
    if (covariance.rows() != covariance.cols()) {
        return false;
    }
    for (int r = 0; r < covariance.rows() - 1; ++r) {
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
