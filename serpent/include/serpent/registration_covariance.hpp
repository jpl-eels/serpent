#ifndef SERPENT_REGISTRATION_COVARIANCE_HPP
#define SERPENT_REGISTRATION_COVARIANCE_HPP

#include <pcl/common/distances.h>
#include <pcl/registration/registration.h>

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <eigen_ext/matrix.hpp>
#include <eigen_ext/precomputed_transform_components.hpp>
#include <pointcloud_tools/point_types.hpp>
#include <pointcloud_tools/registration_utilities.hpp>

/**
 * @brief TODO: There is unfortunately some code duplication here due to the difficulty in systematically separating the
 * different parts of the covariance estimation process. This includes the method (e.g. constant, censi, lls), the model
 * (e.g. point to point, point to plane, and linearised vs non-linear) and how the covariance of a point is modelled
 * (e.g. fixed equal variance per point, covariance in the source/target points, or a covariance matrix).
 */

namespace serpent {

enum class CovarianceEstimationMethod { CONSTANT, CENSI, LLS };

enum class CovarianceEstimationModel {
    POINT_TO_POINT_LINEARISED,
    POINT_TO_POINT_NONLINEAR,
    POINT_TO_PLANE_LINEARISED,
    POINT_TO_PLANE_NONLINEAR
};

enum class PointCovarianceMethod { CONSTANT, VOXEL_SIZE, RANGE, RANGE_BIAS };

bool requires_unit_vectors(const CovarianceEstimationMethod cov_method, const PointCovarianceMethod point_method);

bool requires_unit_vectors(const std::string& cov_method, const std::string& point_method);

CovarianceEstimationMethod to_covariance_estimation_method(const std::string& string);

CovarianceEstimationModel to_covariance_estimation_model(const std::string& string);

PointCovarianceMethod to_point_covariance_method(const std::string& string);

std::string to_string(const CovarianceEstimationMethod method);

std::string to_string(const CovarianceEstimationModel model);

std::string to_string(const PointCovarianceMethod method);

/**
 * @brief Compute the Censi covariance as in censi_covariance(typename pcl::Registration<PointSource, PointTarget,
 * Scalar>&, const std::string&), except here \f$\Sigma_z\f$ is assumed to be a diagonal matrix with point_variance
 * occupying the diagonal elements.
 *
 * @tparam Model
 * @tparam Scalar
 * @param registration
 * @param point_variance
 * @param correspondence_count
 * @return Eigen::Matrix<double, 6, 6>
 */
template<typename Model, typename Scalar>
Eigen::Matrix<double, 6, 6> censi_iid_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double point_variance, int& correspondence_count);

/**
 * @brief Compute the Censi covariance as per Eq (4) of An accurate closed-form estimate of ICP'S covariance (2007):
 *
 * \f[
 *  \Sigma_x = \left(\frac{d^2F}{dx^2}\right)^{-1} \left(\frac{d^2F}{dzdx}\right) \Sigma_z
 *      \left(\frac{d^2F}{dzdx}\right)^T \left(\frac{d^2F}{dx^2}\right)^{-1}
 * \f]
 *
 * In the implementation it is often more efficient to compute:
 *
 * \f[
 *  \Sigma_x = \left(\frac{1}{2}\frac{d^2F}{dx^2}\right)^{-1} \left(\frac{1}{2}\frac{d^2F}{dzdx}\right) \Sigma_z
 *      \left(\frac{1}{2}\frac{d^2F}{dzdx}\right)^T \left(\frac{1}{2}\frac{d^2F}{dx^2}\right)^{-1}
 * \f]
 *
 * \f$\Sigma_z\f$ is block-diagonal where the 3x3 diagonal blocks are computed as (\mathbf{\hat{p}} are unit vectors
 * from sensor origin or each point):
 *
 * \f[
 *  \Sigma_z = \sigma_r^2 \mathbf{\hat{p}}\mathbf{\hat{p}}^T
 * \f]
 *
 * Only usable when Model's PointSource and PointTarget are PointNormalUnit.
 *
 * @tparam Model
 * @tparam Scalar
 * @tparam int anonymous parameter used for compile-time function selection based on Model point types
 * @param registration
 * @param range_variance \f$\sigma_r^2\f$
 * @param correspondence_count
 * @return Eigen::Matrix<double, 6, 6>
 */
template<typename Model, typename Scalar, int>
Eigen::Matrix<double, 6, 6> censi_range_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double range_variance, int& correspondence_count);

template<typename Model, typename Scalar, int>
Eigen::Matrix<double, 6, 6> censi_range_bias_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double range_variance, const double range_bias_variance, int& correspondence_count);

/**
 * @brief Compute the covariance from the Hessian of the linearised system. This is the linear least squares
 * covariance when the residuals are assumed to have i.i.d. Gaussian noise:
 *
 * \f[
 *  \Sigma_x = \sigma_z^2 (\mathbf{A}^T\mathbf{A})^{-1} = \sigma_z^2 \left(\frac{1}{2}\frac{d^2F}{dx^2}\right)^-1
 * \f]
 *
 * Here \f$\sigma_z^2\f$ is the point_variance.
 *
 * @tparam Model
 * @tparam Scalar
 * @param registration
 * @param point_variance
 * @param correspondence_count
 * @return Eigen::Matrix<double, 6, 6>
 */
template<typename Model, typename Scalar>
Eigen::Matrix<double, 6, 6> lls_iid_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double point_variance, int& correspondence_count);

template<typename PointSource_, typename PointTarget_>
class PointToPointIcpNonlinearModel {
public:
    using PointSource = PointSource_;
    using PointTarget = PointTarget_;
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx);

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dx2(const eigen_ext::PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dzdx(
            const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q);
};

template<typename PointSource_, typename PointTarget_>
class PointToPointIcpLinearisedModel {
public:
    using PointSource = PointSource_;
    using PointTarget = PointTarget_;
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx);

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dx2(const eigen_ext::PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dzdx(
            const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q);
};

template<typename PointSource_, typename PointTarget_>
class PointToPlaneIcpNonlinearModel {
public:
    using PointSource = PointSource_;
    using PointTarget = PointTarget_;
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_x)>::value,
            "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_y)>::value,
            "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_z)>::value,
            "normal_z is not a floating point type");

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& half_d2F_dx2,
            Eigen::Matrix<double, 6, 6>& half_d2F_dzdx);

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& half_d2F_dx2);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dx2(const eigen_ext::PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
            const Eigen::Matrix<double, 3, 1>& n);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dzdx(
            const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n);
};

template<typename PointSource_, typename PointTarget_>
class PointToPlaneIcpLinearisedModel {
public:
    using PointSource = PointSource_;
    using PointTarget = PointTarget_;
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_x)>::value,
            "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_y)>::value,
            "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_z)>::value,
            "normal_z is not a floating point type");

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& half_d2F_dx2,
            Eigen::Matrix<double, 6, 6>& half_d2F_dzdx);

    static bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const eigen_ext::PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& half_d2F_dx2);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dx2(const eigen_ext::PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
            const Eigen::Matrix<double, 3, 1>& n);

    static Eigen::Matrix<double, 6, 6> compute_half_d2F_dzdx(
            const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n);
};

}

#include "serpent/impl/registration_covariance.hpp"

#endif
