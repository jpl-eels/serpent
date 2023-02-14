#ifndef EIGEN_EXT_POSE_HPP
#define EIGEN_EXT_POSE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eigen_ext/matrix.hpp"

namespace eigen_ext {

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> cartesian_to_polar(const Scalar x, const Scalar y, const Scalar z);

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> cartesian_to_polar(const Eigen::Matrix<Scalar, 3, 1>& cartesian_point);

/**
 * @brief Change the reference frame of a transform covariance matrix from frame A to frame B:
 *      Cov_B = Adj_{T_B^A} Cov_A (Adj_{T_B^A})^T
 *
 * @tparam Scalar
 * @param covariance_A
 * @param transform_adjoint_B_A
 * @return Eigen::Matrix<Scalar, 6, 6>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> change_covariance_frame(const typename Eigen::Matrix<Scalar, 6, 6>& covariance_A,
        const typename Eigen::Matrix<Scalar, 6, 6>& transform_adjoint_B_A);

/**
 * @brief Wrapper function to change the reference frame of a transform covariance matrix from frame A to frame B.
 *
 * Note: the function computes the adjoint of the transform, so the other function overload should be preferred if
 * using this function multiple times with the same transform.
 *
 * @tparam Scalar
 * @param covariance_A
 * @param transform_B_A
 * @return Eigen::Matrix<Scalar, 6, 6>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> change_covariance_frame(const typename Eigen::Matrix<Scalar, 6, 6>& covariance_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform_B_A);

/**
 * @brief Given a relative transform between any two timestamps in the reference frame of A, and a rigid body transform
 * from some other fixed frame B on the body to A, this function computes the relative transform in the reference frame
 * of B.
 *      T_{B1}^{B2} = T_{B1}^{A1} T_{A1}^{A2} T_{A2}^{B2} =  T_B^A T_{A1}^{A2} T_A^B
 *
 * Previously:
 *      T_{B1}^{B2} = (T_{B2}^{A2} (T_{B1}^{A1} T_{A1}^{A2})^-1 )^-1 = (T_B^A (T_B^A T_{A1}^{A2})^-1 )^-1
 *
 * @tparam Scalar
 * @param relative_transform_A
 * @param rigid_transform_B_A
 * @return Eigen::Transform<Scalar, 3, Eigen::Isometry>
 */
template<typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> change_relative_transform_frame(
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& relative_transform_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_B_A);

template<typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> change_relative_transform_frame(
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& relative_transform_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_B_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_A_B);

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
Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3> compose_transform_covariance(
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& previous_covariance,
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& relative_covariance,
        const Eigen::Transform<Scalar, Dim, Eigen::Isometry>& relative_transform,
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& relative_cross_covariance =
                Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>::Zero());

/**
 * @brief Estimate the constant angular and linear velocities required to transform from pose_1 to pose_2 in time dt.
 * This will relative to pose_1. For example if pose_1 and pose_2 are body frame poses with respect to the world frame,
 * then this function produces the body frame rates, not the world frame rates.
 *
 * @tparam Scalar
 * @param pose_1
 * @param pose_2
 * @param dt
 * @return Eigen::Matrix<Scalar, 6, 1> rates in order of angular then linear, so [w1, w2, w2, v1, v2, v3]
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> linear_rates(const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose_1,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose_2, const Scalar dt);

/**
 * @brief Compute the transform from pose_1 to pose_2.
 *
 * This is a shortcut for pose_1.inverse() * pose_2;
 *
 * @tparam Scalar
 * @param pose_1
 * @param pose_2
 * @return Eigen::Transform<Scalar, 3, Eigen::Isometry>
 */
template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> relative_transform(
        const typename Eigen::Transform<Scalar, Dim, Eigen::Isometry>& pose_1,
        const typename Eigen::Transform<Scalar, Dim, Eigen::Isometry>& pose_2);

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

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> cartesian_to_polar(const Scalar x, const Scalar y, const Scalar z) {
    static_assert(std::is_floating_point<Scalar>::value, "Scalar is not a floating point type");
    const Scalar x2_plus_y2 = x * x + y * y;
    const Scalar r = std::sqrt(x2_plus_y2 + z * z);
    if (x2_plus_y2 == 0.0) {
        throw std::runtime_error("Failed to convert from cartesian to polar coordinates. x and y are zero.");
    }
    const Scalar a_ = std::acos(x / x2_plus_y2);
    const Scalar a = y >= 0.0 ? a_ : 2.0 * M_PI - a_;
    const Scalar e = std::acos(z / r);
    return Eigen::Matrix<Scalar, 3, 1>{r, a, e};
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1> cartesian_to_polar(const Eigen::Matrix<Scalar, 3, 1>& cartesian_point) {
    return cartesian_to_polar<Scalar>(cartesian_point[0], cartesian_point[1], cartesian_point[2]);
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> change_covariance_frame(const typename Eigen::Matrix<Scalar, 6, 6>& covariance_A,
        const typename Eigen::Matrix<Scalar, 6, 6>& transform_adjoint_B_A) {
    return transform_adjoint_B_A * covariance_A * transform_adjoint_B_A.transpose();
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 6> change_covariance_frame(const typename Eigen::Matrix<Scalar, 6, 6>& covariance_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform_B_A) {
    return change_covariance_frame(covariance_A, transform_adjoint(transform_B_A));
}

template<typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> change_relative_transform_frame(
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& relative_transform_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_B_A) {
    // return (rigid_transform_B_A * (rigid_transform_B_A * relative_transform_A).inverse()).inverse();
    return change_relative_transform_frame(relative_transform_A, rigid_transform_B_A, rigid_transform_B_A.inverse());
}

template<typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Isometry> change_relative_transform_frame(
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& relative_transform_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_B_A,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& rigid_transform_A_B) {
    return rigid_transform_B_A * relative_transform_A * rigid_transform_A_B;
}

template<typename Scalar, int Dim>
Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3> compose_transform_covariance(
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& previous_covariance,
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& relative_covariance,
        const Eigen::Transform<Scalar, Dim, Eigen::Isometry>& relative_transform,
        const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1) * 3>& relative_cross_covariance) {
    const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1)* 3> adj = transform_adjoint(relative_transform.inverse());
    const Eigen::Matrix<Scalar, (Dim - 1) * 3, (Dim - 1)* 3> adj_transpose = adj.transpose();
    return adj * previous_covariance * adj_transpose + relative_covariance + relative_cross_covariance * adj_transpose +
           adj * relative_cross_covariance.transpose();
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> linear_rates(const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose_1,
        const typename Eigen::Transform<Scalar, 3, Eigen::Isometry>& pose_2, const Scalar dt) {
    if (dt <= 0.0) {
        throw std::runtime_error("dt must be > 0.0");
    }
    const typename Eigen::Transform<Scalar, 3, Eigen::Isometry> transform = relative_transform(pose_1, pose_2);
    const typename Eigen::AngleAxis<Scalar> rotation{transform.rotation()};
    typename Eigen::Matrix<Scalar, 6, 1> rates;
    rates << rotation.axis() * rotation.angle() / dt, transform.translation() / dt;
    return rates;
}

template<typename Scalar, int Dim>
Eigen::Transform<Scalar, Dim, Eigen::Isometry> relative_transform(
        const typename Eigen::Transform<Scalar, Dim, Eigen::Isometry>& pose_1,
        const typename Eigen::Transform<Scalar, Dim, Eigen::Isometry>& pose_2) {
    return pose_1.inverse() * pose_2;
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
        const Eigen::Matrix<Scalar, 6, 1>& twist) {}

}

#endif
