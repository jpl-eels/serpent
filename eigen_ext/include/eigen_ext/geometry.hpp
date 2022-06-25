#ifndef EIGEN_EXT_POSE_HPP
#define EIGEN_EXT_POSE_HPP

#include "eigen_ext/matrix.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_ext {

/**
 * @brief Given a transform T_a^b (i.e. b in the reference frame of a) and the twist in reference frame of b, compute
 * the twist in reference frame a. Internally, this function calculates and applies the pose adjoint:
 * 
 *      V_a = Ad_{T_a^b} V_b
 * 
 * @tparam Scalar 
 * @param transform 
 * @param twist 
 * @return Eigen::Matrix<Scalar, 6, 1> 
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> change_twist_reference_frame(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform,
        const Eigen::Matrix<Scalar, 6, 1>& twist);

/**
 * @brief Approximation of the covariance of transform composition.
 * 
 * Given the covariance Cov_A^B of a (previous) state T_A^B, and a relative transform T_B^C from that state to a new
 * state (T_A^C = T_A^B T_B^C) with covariance Cov_B^C, the covariance of the new state Cov_A^C is computed. The cross
 * correlation between the two relative transforms Cov_{A,B}^{B,C} can also be supplied to achieve a better estimate as
 * described in Mangelson et al (2019). Otherwise, the approximation is equal to Barfoot et al's (2013).
 * 
 * The covariances are assumed to be in the order of [R1, R2, R3, t1, t2, t3]
 * 
 * References:
 *  - Characterizing the Uncertainty of Jointly Distributed Poses in the Lie Algebra, Mangelson et al (2019)
 *  - Associating Uncertainty With Three-Dimensional Poses for Use in Estimation Problems, Barfoot et al (2013)
 *  - https://gtsam.org/2021/02/23/uncertainties-part3.html Eq (15)
 *  - Modern Robotics, Chapter 3, Park & Lynch
 * 
 * @tparam Derived 
 * @param previous_covariance 
 * @param relative_covariance 
 * @param relative_transform 
 * @return Derived 
 */
template<typename Scalar, int Dim>
Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3> compose_transform_covariance(
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& previous_covariance,
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& relative_covariance,
        const Eigen::Transform<Scalar, Dim, Eigen::Isometry>& relative_transform,
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& relative_cross_covariance = 
        Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>::Zero());

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Translation<Scalar, Dim>& t,
        const Eigen::Quaternion<Scalar>& q);

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Matrix<Scalar, Dim, 1>& t,
        const Eigen::Quaternion<Scalar>& q);

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Matrix<Scalar, Dim, 1>& t,
        const Eigen::Matrix<Scalar, Dim, Dim>& R);

/**
 * @brief The adjoint of a transform T (R, t) in SE(3) as defined by Modern Robotics (Lynch & Park). Note that the
 * adjoint order is [R1, R2, R3, T1, T2, T3], so for example it can be used to change the reference frame of twists in
 * the form [omega, v] by: V_a = Ad_{T_a^b} V_b
 * 
 * @tparam Scalar 
 * @param transform 
 * @return Eigen::Matrix<Scalar, 6, 6> 
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> transform_adjoint(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform);

/* Implementation */

template<typename Scalar, int Dim>
Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3> compose_transform_covariance(
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& previous_covariance,
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& relative_covariance,
        const Eigen::Transform<Scalar, Dim, Eigen::Isometry>& relative_transform,
        const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3>& relative_cross_covariance) {
    const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3> adj = transform_adjoint(relative_transform.inverse());
    const Eigen::Matrix<Scalar, (Dim-1)*3, (Dim-1)*3> adj_transpose = adj.transpose();
    return adj * previous_covariance * adj_transpose + relative_covariance
            + relative_cross_covariance * adj_transpose + adj * relative_cross_covariance.transpose();
}

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Translation<Scalar, Dim>& t,
        const Eigen::Quaternion<Scalar>& q) {
    return t * q;
}

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Matrix<Scalar, Dim, 1>& t,
        const Eigen::Quaternion<Scalar>& q) {
    return to_transform(Eigen::Translation<Scalar, Dim>{t}, q);
}

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> to_transform(const Eigen::Matrix<Scalar, Dim, 1>& t,
        const Eigen::Matrix<Scalar, Dim, Dim>& R) {
    return to_transform(Eigen::Translation<Scalar, Dim>{t}, Eigen::Quaternion<Scalar>{R});
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> transform_adjoint(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform) {
    const Eigen::Matrix<Scalar, 3, 1> t = transform.translation();
    const Eigen::Matrix<Scalar, 3, 3> R = transform.rotation();
    Eigen::Matrix<Scalar, 6, 6> adj;
    adj << R, Eigen::Matrix<Scalar, 3, 3>::Zero(), skew_symmetric(t) * R, R;
    return adj;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> change_twist_reference_frame(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform,
        const Eigen::Matrix<Scalar, 6, 1>& twist) {
    return transform_adjoint(transform) * twist;
}

}

#endif
