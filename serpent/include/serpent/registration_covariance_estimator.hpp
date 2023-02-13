#ifndef SERPENT_REGISTRATION_COVARIANCE_ESTIMATOR_HPP
#define SERPENT_REGISTRATION_COVARIANCE_ESTIMATOR_HPP

#include <pcl/common/distances.h>
#include <pcl/registration/registration.h>

#include <Eigen/Geometry>
#include <pointcloud_tools/registration_utilities.hpp>

namespace serpent {

template<typename Scalar>
class PrecomputedTransformComponents {
public:
    PrecomputedTransformComponents(const Eigen::Matrix<Scalar, 4, 4>& transform);

    PrecomputedTransformComponents(const Eigen::Matrix<Scalar, 3, 1>& r, const Eigen::Matrix<Scalar, 3, 1>& t);

    inline const Eigen::Matrix<Scalar, 4, 4>& T() const;
    inline const Eigen::Matrix<Scalar, 3, 3>& R() const;
    inline const Eigen::Matrix<Scalar, 3, 1>& r() const;
    inline const Eigen::Matrix<Scalar, 3, 1>& t() const;
    inline const Scalar a() const;
    inline const Scalar sina() const;
    inline const Scalar cosa() const;

private:
    Eigen::Matrix<Scalar, 4, 4> T_;
    Eigen::Matrix<Scalar, 3, 3> R_;
    Eigen::Matrix<Scalar, 3, 1> r_;
    Eigen::Matrix<Scalar, 3, 1> t_;
    Scalar a_;
    Scalar sina_;
    Scalar cosa_;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class RegistrationCovarianceEstimator {
public:
    virtual Eigen::Matrix<double, 6, 6> estimate_covariance(
            typename pcl::Registration<PointSource, PointTarget, Scalar>& registration,
            const double point_variance) = 0;

private:
    // Number of correspondences used in last covariance estimation
    int correspondence_count_;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class ConstantCovariance : public RegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
public:
    ConstantCovariance(const Eigen::Matrix<double, 6, 6>& constant_covariance);

    ConstantCovariance(const double rotation_noise, const double translation_noise);

    Eigen::Matrix<double, 6, 6> estimate_covariance(
            typename pcl::Registration<PointSource, PointTarget, Scalar>& registration,
            const double point_variance) override;

protected:
    Eigen::Matrix<double, 6, 6> constant_covariance;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class CorrespondenceRegistrationCovarianceEstimator
    : public RegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");

public:
    Eigen::Matrix<double, 6, 6> estimate_covariance(
            typename pcl::Registration<PointSource, PointTarget, Scalar>& registration,
            const double point_variance) override;

    int correspondence_count() const;

protected:
    virtual bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx) const = 0;

private:
    // Number of correspondences used in last covariance estimation
    int correspondence_count_;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class PointToPointIcpNonlinear
    : public CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
protected:
    bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx) const override;

public:
    Eigen::Matrix<double, 6, 6> compute_d2F_dx2(const PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q) const;

    Eigen::Matrix<double, 6, 6> compute_d2F_dzdx(const PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q) const;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class PointToPointIcpLinearised
    : public CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
protected:
    bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx) const override;

public:
    Eigen::Matrix<double, 6, 6> compute_d2F_dx2(const Eigen::Matrix<double, 3, 1>& r,
            const Eigen::Matrix<double, 3, 1>& t, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q) const;

    Eigen::Matrix<double, 6, 6> compute_d2F_dzdx(const Eigen::Matrix<double, 3, 1>& r,
            const Eigen::Matrix<double, 3, 1>& t, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q) const;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class PointToPlaneIcpNonlinear
    : public CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
    static_assert(std::is_floating_point<decltype(PointTarget::normal_x)>::value,
            "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_y)>::value,
            "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_z)>::value,
            "normal_z is not a floating point type");

protected:
    bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx) const override;

public:
    Eigen::Matrix<double, 6, 6> compute_d2F_dx2(const PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
            const Eigen::Matrix<double, 3, 1>& n) const;

    Eigen::Matrix<double, 6, 6> compute_d2F_dzdx(const PrecomputedTransformComponents<double>& tf,
            const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
            const Eigen::Matrix<double, 3, 1>& n) const;
};

template<typename PointSource, typename PointTarget, typename Scalar = float>
class PointToPlaneIcpLinearised
    : public CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar> {
    static_assert(std::is_floating_point<decltype(PointTarget::normal_x)>::value,
            "normal_x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_y)>::value,
            "normal_y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::normal_z)>::value,
            "normal_z is not a floating point type");

protected:
    bool process_correspondence(const PointSource& source_point, const PointTarget& target_point,
            const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
            Eigen::Matrix<double, 6, 6>& d2F_dzdx) const override;

public:
    Eigen::Matrix<double, 6, 6> compute_d2F_dx2(const Eigen::Matrix<double, 3, 1>& r,
            const Eigen::Matrix<double, 3, 1>& t, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) const;

    Eigen::Matrix<double, 6, 6> compute_d2F_dzdx(const Eigen::Matrix<double, 3, 1>& r,
            const Eigen::Matrix<double, 3, 1>& t, const Eigen::Matrix<double, 3, 1>& p,
            const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) const;
};

/** Implementation */

template<typename Scalar>
PrecomputedTransformComponents<Scalar>::PrecomputedTransformComponents(const Eigen::Matrix<Scalar, 4, 4>& transform)
    : T_(transform) {
    R_ = T_.template block<3, 3>(0, 0);
    const Eigen::AngleAxis<Scalar> angleaxis{R_};
    a_ = angleaxis.angle();
    sina_ = std::sin(a_);
    cosa_ = std::cos(a_);
    r_ = angleaxis.axis() * a_;
    t_ = T_.template block<3, 1>(0, 3);
}

template<typename Scalar>
PrecomputedTransformComponents<Scalar>::PrecomputedTransformComponents(const Eigen::Matrix<Scalar, 3, 1>& r,
        const Eigen::Matrix<Scalar, 3, 1>& t)
    : r_(r),
      t_(t) {
    a_ = r_.norm();
    const Eigen::AngleAxis<Scalar> angleaxis{a_, r_ / a_};
    R_ = angleaxis.toRotationMatrix();
    sina_ = std::sin(a_);
    cosa_ = std::cos(a_);
    T_ << R_, t_, Eigen::Matrix<Scalar, 1, 3>::Zero(), static_cast<Scalar>(1);
}

template<typename Scalar>
inline const Eigen::Matrix<Scalar, 4, 4>& PrecomputedTransformComponents<Scalar>::T() const {
    return T_;
}

template<typename Scalar>
inline const Eigen::Matrix<Scalar, 3, 3>& PrecomputedTransformComponents<Scalar>::R() const {
    return R_;
}

template<typename Scalar>
inline const Eigen::Matrix<Scalar, 3, 1>& PrecomputedTransformComponents<Scalar>::r() const {
    return r_;
}

template<typename Scalar>
inline const Eigen::Matrix<Scalar, 3, 1>& PrecomputedTransformComponents<Scalar>::t() const {
    return t_;
}

template<typename Scalar>
inline const Scalar PrecomputedTransformComponents<Scalar>::a() const {
    return a_;
}

template<typename Scalar>
inline const Scalar PrecomputedTransformComponents<Scalar>::sina() const {
    return sina_;
}

template<typename Scalar>
inline const Scalar PrecomputedTransformComponents<Scalar>::cosa() const {
    return cosa_;
}

template<typename PointSource, typename PointTarget, typename Scalar>
ConstantCovariance<PointSource, PointTarget, Scalar>::ConstantCovariance(
        const Eigen::Matrix<double, 6, 6>& constant_covariance)
    : constant_covariance(constant_covariance) {}

template<typename PointSource, typename PointTarget, typename Scalar>
ConstantCovariance<PointSource, PointTarget, Scalar>::ConstantCovariance(const double rotation_noise,
        const double translation_noise) {
    constant_covariance << Eigen::Matrix<double, 3, 3>::Identity() * rotation_noise * rotation_noise,
            Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Zero(),
            Eigen::Matrix<double, 3, 3>::Identity() * translation_noise * translation_noise;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> ConstantCovariance<PointSource, PointTarget, Scalar>::estimate_covariance(
        typename pcl::Registration<PointSource, PointTarget, Scalar>&, const double) {
    return constant_covariance;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6>
CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar>::estimate_covariance(
        typename pcl::Registration<PointSource, PointTarget, Scalar>& registration, const double point_variance) {
    // Setup
    PrecomputedTransformComponents<double> tf{registration.getFinalTransformation().template cast<double>()};
    const auto source_cloud = registration.getInputSource();
    const auto target_cloud = registration.getInputTarget();

    // Compute correspondences
    const auto correspondences = pct::compute_registration_correspondences(registration);
    correspondence_count_ = correspondences->size();

    // Interate over correspondences to build hessian and d2F_dzdx
    Eigen::Matrix<double, 6, 6> d2F_dx2 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, Eigen::Dynamic> d2F_dzdx(6, 6 * correspondences->size());
    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        const auto& correspondence = (*correspondences)[i];
        const PointSource& source_point = source_cloud->at(correspondence.index_query);
        const PointTarget& target_point = target_cloud->at(correspondence.index_match);
        Eigen::Matrix<double, 6, 6> d2F_dx2_i, d2F_dzdx_i;
        if (process_correspondence(source_point, target_point, tf, d2F_dx2_i, d2F_dzdx_i)) {
            const double d2F_dx2_i_max_coeff = d2F_dx2_i.maxCoeff();
            const double d2F_dzdx_i_max_coeff = d2F_dzdx_i.maxCoeff();
            d2F_dx2 += d2F_dx2_i;
            d2F_dzdx.block(0, 6 * i, 6, 6) = d2F_dzdx_i;
        }
    }
    const Eigen::Matrix<double, 6, 6> d2F_dx2_inv = d2F_dx2.inverse();
    // Use brackets to ensure that the Nx6K * 6KxN multiplication occurs first.
    // The point_variance multiplication is also moved outside the centre, since it is a scalar product in this case.
    return point_variance * d2F_dx2_inv * (d2F_dzdx * d2F_dzdx.transpose()) * d2F_dx2_inv;
}

template<typename PointSource, typename PointTarget, typename Scalar>
int CorrespondenceRegistrationCovarianceEstimator<PointSource, PointTarget, Scalar>::correspondence_count() const {
    return correspondence_count_;
}

template<typename PointSource, typename PointTarget, typename Scalar>
bool PointToPointIcpNonlinear<PointSource, PointTarget, Scalar>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& d2F_dx2, Eigen::Matrix<double, 6, 6>& d2F_dzdx) const {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    d2F_dx2 = compute_d2F_dx2(tf, p, q);
    d2F_dzdx = compute_d2F_dzdx(tf, p, q);
    return true;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPointIcpNonlinear<PointSource, PointTarget, Scalar>::compute_d2F_dx2(
        const PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) const {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx4 = rx * rx * rx * rx;      // rx4
    const double ry4 = ry * ry * ry * ry;      // ry4
    const double rz4 = rz * rz * rz * rz;      // rz4
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx2
    const double ry2 = ry * ry;                // ry2
    const double rz2 = rz * rz;                // rz2
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double a5 = a4 * tf.a();             // a5
    const double a6 = a5 * tf.a();             // a6
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;  // cam1

    Eigen::Matrix<double, 6, 6> d2F_dx2;
    d2F_dx2(0, 0) =
            (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                    px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                 (rx2ry * sa) / a3) +
                    pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                            (rx * rz * sa) / a3 + (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            (2 * pz *
                            (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 - (rz2 * sa) / a3 - (2 * rz2 * cam1) / a4 -
                                    (ca * rx2rz2) / a4 + (5 * rx2rz2 * sa) / a5 + (8 * rx2rz2 * cam1) / a6) -
                    2 * px *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (8 * rx3rz * cam1) / a6 + (3 * rx * rz * sa) / a3 +
                                    (3 * ca * rx2ry) / a4 + (6 * rx * rz * cam1) / a4 + (ca * rx3rz) / a4 +
                                    (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 - (5 * rx3rz * sa) / a5) +
                    2 * py *
                            ((3 * rx * sa) / a3 + (3 * ca * rx3) / a4 + (rx3 * sa) / a3 - (3 * rx3 * sa) / a5 -
                                    (3 * ca * rx) / a2 - (ry * rz * sa) / a3 - (2 * ry * rz * cam1) / a4 -
                                    (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) -
            (2 * px *
                            (sa / a + (2 * cam1) / a2 + (ca * rx2) / a2 - (ca * rx4) / a4 - (6 * rx2 * sa) / a3 +
                                    (5 * rx4 * sa) / a5 - (10 * rx2 * cam1) / a4 + (8 * rx4 * cam1) / a6) -
                    2 * py *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (8 * rx3ry * cam1) / a6 + (3 * rx * ry * sa) / a3 +
                                    (6 * rx * ry * cam1) / a4 + (ca * rx3ry) / a4 + (3 * ca * rx2rz) / a4 -
                                    (5 * rx3ry * sa) / a5 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5) +
                    2 * pz *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (8 * rx3rz * cam1) / a6 - (3 * rx * rz * sa) / a3 +
                                    (3 * ca * rx2ry) / a4 - (6 * rx * rz * cam1) / a4 - (ca * rx3rz) / a4 +
                                    (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 + (5 * rx3rz * sa) / a5)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) -
            (2 * py *
                            (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 - (ry2 * sa) / a3 - (2 * ry2 * cam1) / a4 -
                                    (ca * rx2ry2) / a4 + (5 * rx2ry2 * sa) / a5 + (8 * rx2ry2 * cam1) / a6) +
                    2 * px *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (8 * rx3ry * cam1) / a6 - (3 * rx * ry * sa) / a3 -
                                    (6 * rx * ry * cam1) / a4 - (ca * rx3ry) / a4 + (3 * ca * rx2rz) / a4 +
                                    (5 * rx3ry * sa) / a5 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5) -
                    2 * pz *
                            ((3 * rx * sa) / a3 + (3 * ca * rx3) / a4 + (rx3 * sa) / a3 - (3 * rx3 * sa) / a5 -
                                    (3 * ca * rx) / a2 + (ry * rz * sa) / a3 + (2 * ry * rz * cam1) / a4 +
                                    (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) +
            (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                    px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                 (rx2rz * sa) / a3) +
                    py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                            (rx * ry * sa) / a3 + (rx2rz * sa) / a3) +
                            2 * py *
                                    (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                          (rx2ry * sa) / a3) +
                    pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                 (rx2rz * sa) / a3) -
                    px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                            (rx * rz * sa) / a3 + (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                            (rx * ry * sa) / a3 + (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4));
    d2F_dx2(0, 1) =
            (2 * px *
                            ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                    (8 * rx2ry2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * py *
                            ((8 * rxry3 * cam1) / a6 + (ca * rx * ry) / a2 - (3 * rx * ry * sa) / a3 -
                                    (4 * rx * ry * cam1) / a4 - (ca * rxry3) / a4 + (5 * rxry3 * sa) / a5) +
                    2 * pz *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (rx * rz * sa) / a3 + (3 * ca * rx2ry) / a4 +
                                    (2 * rx * rz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 +
                                    (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) -
            (2 * px *
                            ((8 * rx3ry * cam1) / a6 + (ca * rx * ry) / a2 - (3 * rx * ry * sa) / a3 -
                                    (4 * rx * ry * cam1) / a4 - (ca * rx3ry) / a4 + (5 * rx3ry * sa) / a5) -
                    2 * py *
                            ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                    (8 * rx2ry2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * pz *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (ry * rz * sa) / a3 + (3 * ca * rxry2) / a4 -
                                    (2 * ry * rz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 -
                                    (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) +
            (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                    2 * px *
                            ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                    (rx2rz * sa) / a3) +
                    2 * py *
                            (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                         (rx * ry * sa) / a3 + (ry2rz * sa) / a3) +
                            px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                         (rxryrz * sa) / a3)) +
            (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                    py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                 (rxry2 * sa) / a3) +
                    pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                            (rx * rz * sa) / a3 + (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                            (rx * ry * sa) / a3 + (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                    2 * px *
                            ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                    (rx2ry * sa) / a3) +
                    2 * pz *
                            ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                  (rxry2 * sa) / a3) +
                            pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                         (rx * ry * sa) / a3 + (ry2rz * sa) / a3) -
                            py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) -
            (2 * pz *
                            ((ca * rx * ry) / a2 - (rx * ry * sa) / a3 - (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 +
                                    (8 * rxryrz2 * cam1) / a6) -
                    2 * px *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (ry * rz * sa) / a3 + (3 * ca * rxry2) / a4 +
                                    (2 * ry * rz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 +
                                    (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6) +
                    2 * py *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (rx * rz * sa) / a3 + (3 * ca * rx2ry) / a4 -
                                    (2 * rx * rz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 -
                                    (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2));
    d2F_dx2(0, 2) =
            (2 * pz *
                            ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                    (8 * rx2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * px *
                            ((8 * rx3rz * cam1) / a6 + (ca * rx * rz) / a2 - (3 * rx * rz * sa) / a3 -
                                    (4 * rx * rz * cam1) / a4 - (ca * rx3rz) / a4 + (5 * rx3rz * sa) / a5) +
                    2 * py *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (ry * rz * sa) / a3 + (3 * ca * rxrz2) / a4 +
                                    (2 * ry * rz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 +
                                    (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) -
            (2 * pz *
                            ((8 * rxrz3 * cam1) / a6 + (ca * rx * rz) / a2 - (3 * rx * rz * sa) / a3 -
                                    (4 * rx * rz * cam1) / a4 - (ca * rxrz3) / a4 + (5 * rxrz3 * sa) / a5) -
                    2 * px *
                            ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                    (8 * rx2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * py *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (rx * ry * sa) / a3 - (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 -
                                    (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) +
            (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                    2 * px *
                            ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                    (rx2ry * sa) / a3) +
                    2 * pz *
                            ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                         (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) +
                            px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                         (rxryrz * sa) / a3)) +
            (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                    pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                 (rxrz2 * sa) / a3) +
                    py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                            (rx * rz * sa) / a3 + (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                            (rx * ry * sa) / a3 + (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                    2 * px *
                            ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                    (rx2rz * sa) / a3) +
                    2 * py *
                            (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                  (rxrz2 * sa) / a3) +
                            py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                         (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) -
                            pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) -
            (2 * py *
                            ((ca * rx * rz) / a2 - (rx * rz * sa) / a3 - (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 +
                                    (8 * rxry2rz * cam1) / a6) +
                    2 * px *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (ry * rz * sa) / a3 + (3 * ca * rxrz2) / a4 -
                                    (2 * ry * rz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 -
                                    (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6) -
                    2 * pz *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (rx * ry * sa) / a3 + (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 +
                                    (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2));
    d2F_dx2(0, 3) = 2 * py *
                            ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                    (rx2ry * sa) / a3) +
                    2 * pz *
                            ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                    (rx2rz * sa) / a3) -
                    2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4);
    d2F_dx2(0, 4) =
            2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
            2 * px *
                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                            (rx2ry * sa) / a3) +
            2 * pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(0, 5) =
            2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
            2 * px *
                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                            (rx2rz * sa) / a3) +
            2 * py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(1, 0) =
            (2 * px *
                            ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                    (8 * rx2ry2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * py *
                            ((8 * rxry3 * cam1) / a6 + (ca * rx * ry) / a2 - (3 * rx * ry * sa) / a3 -
                                    (4 * rx * ry * cam1) / a4 - (ca * rxry3) / a4 + (5 * rxry3 * sa) / a5) +
                    2 * pz *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (rx * rz * sa) / a3 + (3 * ca * rx2ry) / a4 +
                                    (2 * rx * rz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 +
                                    (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) -
            (2 * px *
                            ((8 * rx3ry * cam1) / a6 + (ca * rx * ry) / a2 - (3 * rx * ry * sa) / a3 -
                                    (4 * rx * ry * cam1) / a4 - (ca * rx3ry) / a4 + (5 * rx3ry * sa) / a5) -
                    2 * py *
                            ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                    (8 * rx2ry2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * pz *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (ry * rz * sa) / a3 + (3 * ca * rxry2) / a4 -
                                    (2 * ry * rz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 -
                                    (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) +
            (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                    px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                 (rx2rz * sa) / a3) +
                    py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                            (rx * ry * sa) / a3 + (ry2rz * sa) / a3) +
                            2 * px *
                                    ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                    2 * py *
                            ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                    (rxry2 * sa) / a3) +
                    2 * pz *
                            (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                  (rx2ry * sa) / a3) +
                            pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                         (rx * ry * sa) / a3 + (rx2rz * sa) / a3) -
                            px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                    px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                 (rx2ry * sa) / a3) +
                    pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                            (ry * rz * sa) / a3 + (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                            (rx * ry * sa) / a3 + (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) -
            (2 * pz *
                            ((ca * rx * ry) / a2 - (rx * ry * sa) / a3 - (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 +
                                    (8 * rxryrz2 * cam1) / a6) -
                    2 * px *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (ry * rz * sa) / a3 + (3 * ca * rxry2) / a4 +
                                    (2 * ry * rz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 +
                                    (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6) +
                    2 * py *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (rx * rz * sa) / a3 + (3 * ca * rx2ry) / a4 -
                                    (2 * rx * rz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 -
                                    (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2));
    d2F_dx2(1, 1) =
            (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                    py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                 (rxry2 * sa) / a3) +
                    pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                            2 * py *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 +
                                            (ry * rz * sa) / a3 + (rxry2 * sa) / a3) +
                            2 * pz *
                                    (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            (2 * pz *
                            (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 - (rz2 * sa) / a3 - (2 * rz2 * cam1) / a4 -
                                    (ca * ry2rz2) / a4 + (5 * ry2rz2 * sa) / a5 + (8 * ry2rz2 * cam1) / a6) -
                    2 * px *
                            ((3 * ry * sa) / a3 + (3 * ca * ry3) / a4 + (ry3 * sa) / a3 - (3 * ry3 * sa) / a5 -
                                    (3 * ca * ry) / a2 + (rx * rz * sa) / a3 + (2 * rx * rz * cam1) / a4 +
                                    (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6) +
                    2 * py *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (8 * ry3rz * cam1) / a6 - (3 * ry * rz * sa) / a3 +
                                    (3 * ca * rxry2) / a4 - (6 * ry * rz * cam1) / a4 - (ca * ry3rz) / a4 +
                                    (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 + (5 * ry3rz * sa) / a5)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) -
            (2 * py *
                            (sa / a + (2 * cam1) / a2 + (ca * ry2) / a2 - (ca * ry4) / a4 - (6 * ry2 * sa) / a3 +
                                    (5 * ry4 * sa) / a5 - (10 * ry2 * cam1) / a4 + (8 * ry4 * cam1) / a6) +
                    2 * px *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (8 * rxry3 * cam1) / a6 - (3 * rx * ry * sa) / a3 -
                                    (6 * rx * ry * cam1) / a4 - (ca * rxry3) / a4 + (3 * ca * ry2rz) / a4 +
                                    (5 * rxry3 * sa) / a5 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5) -
                    2 * pz *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (8 * ry3rz * cam1) / a6 + (3 * ry * rz * sa) / a3 +
                                    (3 * ca * rxry2) / a4 + (6 * ry * rz * cam1) / a4 + (ca * ry3rz) / a4 +
                                    (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 - (5 * ry3rz * sa) / a5)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) -
            (2 * px *
                            (sa / a + (ca * ry2) / a2 - (rx2 * sa) / a3 - (ry2 * sa) / a3 - (2 * rx2 * cam1) / a4 -
                                    (ca * rx2ry2) / a4 + (5 * rx2ry2 * sa) / a5 + (8 * rx2ry2 * cam1) / a6) -
                    2 * py *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (8 * rxry3 * cam1) / a6 + (3 * rx * ry * sa) / a3 +
                                    (6 * rx * ry * cam1) / a4 + (ca * rxry3) / a4 + (3 * ca * ry2rz) / a4 -
                                    (5 * rxry3 * sa) / a5 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5) +
                    2 * pz *
                            ((3 * ry * sa) / a3 + (3 * ca * ry3) / a4 + (ry3 * sa) / a3 - (3 * ry3 * sa) / a5 -
                                    (3 * ca * ry) / a2 - (rx * rz * sa) / a3 - (2 * rx * rz * cam1) / a4 -
                                    (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) +
            (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                    py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                 (ry2rz * sa) / a3) +
                    px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                            (rx * ry * sa) / a3 + (ry2rz * sa) / a3) +
                            2 * px *
                                    ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                          (rxry2 * sa) / a3) +
                    pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                 (ry2rz * sa) / a3) -
                    py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                            (ry * rz * sa) / a3 + (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                            (rx * ry * sa) / a3 + (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4));
    d2F_dx2(1, 2) =
            (2 * py *
                            ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                    (8 * ry2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * pz *
                            ((8 * ryrz3 * cam1) / a6 + (ca * ry * rz) / a2 - (3 * ry * rz * sa) / a3 -
                                    (4 * ry * rz * cam1) / a4 - (ca * ryrz3) / a4 + (5 * ryrz3 * sa) / a5) +
                    2 * px *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (rx * ry * sa) / a3 + (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 +
                                    (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) -
            (2 * py *
                            ((8 * ry3rz * cam1) / a6 + (ca * ry * rz) / a2 - (3 * ry * rz * sa) / a3 -
                                    (4 * ry * rz * cam1) / a4 - (ca * ry3rz) / a4 + (5 * ry3rz * sa) / a5) -
                    2 * pz *
                            ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                    (8 * ry2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * px *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (rx * rz * sa) / a3 - (2 * rx * rz * cam1) / a4 +
                                    (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 -
                                    (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) +
            (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                    2 * py *
                            ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                    (rxry2 * sa) / a3) +
                    2 * pz *
                            (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                         (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                         (rxryrz * sa) / a3)) +
            (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                    pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                 (ryrz2 * sa) / a3) +
                    px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                            (ry * rz * sa) / a3 + (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                            (rx * ry * sa) / a3 + (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                    2 * py *
                            ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                    (ry2rz * sa) / a3) +
                    2 * px *
                            ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                  (rxrz2 * sa) / a3) +
                            py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                         (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) -
                            pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) -
            (2 * px *
                            ((ca * ry * rz) / a2 - (ry * rz * sa) / a3 - (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 +
                                    (8 * rx2ryrz * cam1) / a6) -
                    2 * py *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (rx * rz * sa) / a3 + (2 * rx * rz * cam1) / a4 +
                                    (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 +
                                    (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6) +
                    2 * pz *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (rx * ry * sa) / a3 - (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 -
                                    (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2));
    d2F_dx2(1, 3) =
            2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
            2 * py *
                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                            (rxry2 * sa) / a3) +
            2 * pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(1, 4) = 2 * px *
                            ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                    (rxry2 * sa) / a3) +
                    2 * pz *
                            ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                    (ry2rz * sa) / a3) -
                    2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4);
    d2F_dx2(1, 5) =
            2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
            2 * py *
                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                            (ry2rz * sa) / a3) +
            2 * px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(2, 0) =
            (2 * pz *
                            ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                    (8 * rx2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * px *
                            ((8 * rx3rz * cam1) / a6 + (ca * rx * rz) / a2 - (3 * rx * rz * sa) / a3 -
                                    (4 * rx * rz * cam1) / a4 - (ca * rx3rz) / a4 + (5 * rx3rz * sa) / a5) +
                    2 * py *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (ry * rz * sa) / a3 + (3 * ca * rxrz2) / a4 +
                                    (2 * ry * rz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 +
                                    (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) -
            (2 * pz *
                            ((8 * rxrz3 * cam1) / a6 + (ca * rx * rz) / a2 - (3 * rx * rz * sa) / a3 -
                                    (4 * rx * rz * cam1) / a4 - (ca * rxrz3) / a4 + (5 * rxrz3 * sa) / a5) -
                    2 * px *
                            ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                    (8 * rx2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * py *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (rx * ry * sa) / a3 - (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 -
                                    (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) +
            (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                    px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                 (rx2ry * sa) / a3) +
                    pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                            (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) +
                            2 * px *
                                    (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                    2 * pz *
                            ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                    (rxrz2 * sa) / a3) +
                    2 * py *
                            ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                  (rx2ry * sa) / a3) +
                            pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 -
                                         (rx * ry * sa) / a3 + (rx2rz * sa) / a3) -
                            px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                    px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                 (rx2rz * sa) / a3) +
                    py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 +
                                            (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                            (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) -
            (2 * py *
                            ((ca * rx * rz) / a2 - (rx * rz * sa) / a3 - (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 +
                                    (8 * rxry2rz * cam1) / a6) +
                    2 * px *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (ry * rz * sa) / a3 + (3 * ca * rxrz2) / a4 -
                                    (2 * ry * rz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 -
                                    (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6) -
                    2 * pz *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (rx * ry * sa) / a3 + (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 +
                                    (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2));
    d2F_dx2(2, 1) =
            (2 * py *
                            ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                    (8 * ry2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                    (3 * rxryrz * sa) / a5) -
                    2 * pz *
                            ((8 * ryrz3 * cam1) / a6 + (ca * ry * rz) / a2 - (3 * ry * rz * sa) / a3 -
                                    (4 * ry * rz * cam1) / a4 - (ca * ryrz3) / a4 + (5 * ryrz3 * sa) / a5) +
                    2 * px *
                            ((rz * sa) / a3 - (ca * rz) / a2 + (rx * ry * sa) / a3 + (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 +
                                    (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) -
            (2 * py *
                            ((8 * ry3rz * cam1) / a6 + (ca * ry * rz) / a2 - (3 * ry * rz * sa) / a3 -
                                    (4 * ry * rz * cam1) / a4 - (ca * ry3rz) / a4 + (5 * ry3rz * sa) / a5) -
                    2 * pz *
                            ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                    (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                    (8 * ry2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 + (rxryrz * sa) / a3 -
                                    (3 * rxryrz * sa) / a5) +
                    2 * px *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (rx * rz * sa) / a3 - (2 * rx * rz * cam1) / a4 +
                                    (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 -
                                    (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) +
            (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                    py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                 (rxry2 * sa) / a3) +
                    pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                            (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                    2 * pz *
                            ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                    (ryrz2 * sa) / a3) +
                    2 * px *
                            (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                    (rxryrz * sa) / a3)) *
                    (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                  (rxry2 * sa) / a3) +
                            pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 +
                                         (rx * ry * sa) / a3 + (ry2rz * sa) / a3) -
                            py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                    py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                 (ry2rz * sa) / a3) +
                    px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 +
                                            (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                            (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) -
            (2 * px *
                            ((ca * ry * rz) / a2 - (ry * rz * sa) / a3 - (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 +
                                    (8 * rx2ryrz * cam1) / a6) -
                    2 * py *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (rx * rz * sa) / a3 + (2 * rx * rz * cam1) / a4 +
                                    (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 +
                                    (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6) +
                    2 * pz *
                            ((rz * sa) / a3 - (ca * rz) / a2 - (rx * ry * sa) / a3 - (2 * rx * ry * cam1) / a4 +
                                    (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 -
                                    (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2));
    d2F_dx2(2, 2) =
            (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                    pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                 (rxrz2 * sa) / a3) +
                    py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 -
                                            (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            (2 * py *
                            (sa / a + (ca * rz2) / a2 - (ry2 * sa) / a3 - (rz2 * sa) / a3 - (2 * ry2 * cam1) / a4 -
                                    (ca * ry2rz2) / a4 + (5 * ry2rz2 * sa) / a5 + (8 * ry2rz2 * cam1) / a6) +
                    2 * px *
                            ((3 * rz * sa) / a3 + (3 * ca * rz3) / a4 + (rz3 * sa) / a3 - (3 * rz3 * sa) / a5 -
                                    (3 * ca * rz) / a2 - (rx * ry * sa) / a3 - (2 * rx * ry * cam1) / a4 -
                                    (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6) -
                    2 * pz *
                            ((rx * sa) / a3 - (ca * rx) / a2 - (8 * ryrz3 * cam1) / a6 + (3 * ry * rz * sa) / a3 +
                                    (3 * ca * rxrz2) / a4 + (6 * ry * rz * cam1) / a4 + (ca * ryrz3) / a4 +
                                    (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 - (5 * ryrz3 * sa) / a5)) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rx * ry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ry * rz * cam1) / a2)) -
            (2 * pz *
                            (sa / a + (2 * cam1) / a2 + (ca * rz2) / a2 - (ca * rz4) / a4 - (6 * rz2 * sa) / a3 +
                                    (5 * rz4 * sa) / a5 - (10 * rz2 * cam1) / a4 + (8 * rz4 * cam1) / a6) -
                    2 * px *
                            ((ry * sa) / a3 - (ca * ry) / a2 - (8 * rxrz3 * cam1) / a6 + (3 * rx * rz * sa) / a3 +
                                    (6 * rx * rz * cam1) / a4 + (ca * rxrz3) / a4 + (3 * ca * ryrz2) / a4 -
                                    (5 * rxrz3 * sa) / a5 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5) +
                    2 * py *
                            ((rx * sa) / a3 - (ca * rx) / a2 + (8 * ryrz3 * cam1) / a6 - (3 * ry * rz * sa) / a3 +
                                    (3 * ca * rxrz2) / a4 - (6 * ry * rz * cam1) / a4 - (ca * ryrz3) / a4 +
                                    (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 + (5 * ryrz3 * sa) / a5)) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rx * rz * cam1) / a2) +
                            py * ((rx * sa) / a - (ry * rz * cam1) / a2)) -
            (2 * px *
                            (sa / a + (ca * rz2) / a2 - (rx2 * sa) / a3 - (rz2 * sa) / a3 - (2 * rx2 * cam1) / a4 -
                                    (ca * rx2rz2) / a4 + (5 * rx2rz2 * sa) / a5 + (8 * rx2rz2 * cam1) / a6) -
                    2 * py *
                            ((3 * rz * sa) / a3 + (3 * ca * rz3) / a4 + (rz3 * sa) / a3 - (3 * rz3 * sa) / a5 -
                                    (3 * ca * rz) / a2 + (rx * ry * sa) / a3 + (2 * rx * ry * cam1) / a4 +
                                    (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6) +
                    2 * pz *
                            ((ry * sa) / a3 - (ca * ry) / a2 + (8 * rxrz3 * cam1) / a6 - (3 * rx * rz * sa) / a3 -
                                    (6 * rx * rz * cam1) / a4 - (ca * rxrz3) / a4 + (3 * ca * ryrz2) / a4 +
                                    (5 * rxrz3 * sa) / a5 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5)) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rx * ry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rx * rz * cam1) / a2)) +
            (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                    pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                 (ryrz2 * sa) / a3) +
                    px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) *
                    (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 +
                                            (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) +
                            2 * px *
                                    (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                          (rxrz2 * sa) / a3) +
                    py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                 (ryrz2 * sa) / a3) -
                    pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 +
                                            (ry * rz * sa) / a3 + (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 -
                                            (rx * rz * sa) / a3 + (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4));
    d2F_dx2(2, 3) =
            2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
            2 * pz *
                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                            (rxrz2 * sa) / a3) +
            2 * py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(2, 4) =
            2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
            2 * pz *
                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                            (ryrz2 * sa) / a3) +
            2 * px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(2, 5) = 2 * px *
                            ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                    (rxrz2 * sa) / a3) +
                    2 * py *
                            ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                    (ryrz2 * sa) / a3) -
                    2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4);
    d2F_dx2(3, 0) = 2 * py *
                            ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                                    (rx2ry * sa) / a3) +
                    2 * pz *
                            ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                                    (rx2rz * sa) / a3) -
                    2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4);
    d2F_dx2(3, 1) =
            2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
            2 * py *
                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                            (rxry2 * sa) / a3) +
            2 * pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(3, 2) =
            2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
            2 * pz *
                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                            (rxrz2 * sa) / a3) +
            2 * py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(3, 3) = 2;
    d2F_dx2(3, 4) = 0;
    d2F_dx2(3, 5) = 0;
    d2F_dx2(4, 0) =
            2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
            2 * px *
                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                            (rx2ry * sa) / a3) +
            2 * pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(4, 1) = 2 * px *
                            ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ry * rz) / a2 - (ry * rz * sa) / a3 +
                                    (rxry2 * sa) / a3) +
                    2 * pz *
                            ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                                    (ry2rz * sa) / a3) -
                    2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4);
    d2F_dx2(4, 2) =
            2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
            2 * pz *
                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rx * rz) / a2 + (rx * rz * sa) / a3 +
                            (ryrz2 * sa) / a3) +
            2 * px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(4, 3) = 0;
    d2F_dx2(4, 4) = 2;
    d2F_dx2(4, 5) = 0;
    d2F_dx2(5, 0) =
            2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
            2 * px *
                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rx * ry) / a2 + (rx * ry * sa) / a3 +
                            (rx2rz * sa) / a3) +
            2 * py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(5, 1) =
            2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
            2 * py *
                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rx * ry) / a2 - (rx * ry * sa) / a3 +
                            (ry2rz * sa) / a3) +
            2 * px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dx2(5, 2) = 2 * px *
                            ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ry * rz) / a2 + (ry * rz * sa) / a3 +
                                    (rxrz2 * sa) / a3) +
                    2 * py *
                            ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rx * rz) / a2 - (rx * rz * sa) / a3 +
                                    (ryrz2 * sa) / a3) -
                    2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4);
    d2F_dx2(5, 3) = 0;
    d2F_dx2(5, 4) = 0;
    d2F_dx2(5, 5) = 2;
    return d2F_dx2;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPointIcpNonlinear<PointSource, PointTarget, Scalar>::compute_d2F_dzdx(
        const PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) const {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double rx2ry = rx * rx * ry;   // rx2ry
    const double rx2rz = rx * rx * rz;   // rx2rz
    const double rxry2 = rx * ry * ry;   // rxry2
    const double ry2rz = ry * ry * rz;   // ry2rz
    const double rxrz2 = rx * rz * rz;   // rxrz2
    const double ryrz2 = ry * rz * rz;   // ryrz2
    const double rxryrz = rx * ry * rz;  // rxryrz
    const double rx3 = rx * rx * rx;     // rx3
    const double ry3 = ry * ry * ry;     // ry3
    const double rz3 = rz * rz * rz;     // rz3
    const double rx2 = rx * rx;          // rx2
    const double ry2 = ry * ry;          // ry2
    const double rz2 = rz * rz;          // rz2
    const double rxry = rx * ry;         // rxry
    const double rxrz = rx * rz;         // rxrz
    const double ryrz = ry * rz;         // ryrz
    const double a = tf.a();             // a
    const double a2 = a * tf.a();        // a2
    const double a3 = a2 * tf.a();       // a3
    const double a4 = a3 * tf.a();       // a4
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;  // cam1

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dzdx;
    d2F_dzdx(0, 0) =
            ((rz * sa) / a - (rxry * cam1) / a2) *
                    (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((2 * rx * sa) / a + (4 * rx * cam1) / a2 - (2 * rx3 * sa) / a3 - (4 * rx3 * cam1) / a4) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) -
            ((ry * sa) / a + (rxrz * cam1) / a2) *
                    (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) +
                            2 * py *
                                    (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (ca - (rx2 * cam1) / a2) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            ((4 * rx2ry * cam1) / a4 - (2 * ry * cam1) / a2 + (2 * ca * rxrz) / a2 - (2 * rxrz * sa) / a3 +
                    (2 * rx2ry * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
            ((4 * rx2rz * cam1) / a4 - (2 * rz * cam1) / a2 - (2 * ca * rxry) / a2 + (2 * rxry * sa) / a3 +
                    (2 * rx2rz * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(0, 1) =
            ((2 * sa) / a + (2 * ca * rx2) / a2 - (2 * rx2 * sa) / a3 + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2)) +
            ((rx * sa) / a - (ryrz * cam1) / a2) *
                    (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) +
                            2 * py *
                                    (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((rz * sa) / a + (rxry * cam1) / a2) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (ca - (ry2 * cam1) / a2) * (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                               2 * px *
                                                       ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                               (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                               2 * pz *
                                                       ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * rxry2 * cam1) / a4 - (2 * rx * sa) / a + (2 * rxry2 * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
            ((4 * rx2ry * cam1) / a4 - (2 * ry * cam1) / a2 - (2 * ca * rxrz) / a2 + (2 * rxrz * sa) / a3 +
                    (2 * rx2ry * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2));
    d2F_dzdx(0, 2) =
            ((2 * rx2 * sa) / a3 - (2 * ca * rx2) / a2 - (2 * sa) / a + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) -
            ((rx * sa) / a + (ryrz * cam1) / a2) *
                    (2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                            2 * px *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            ((ry * sa) / a - (rxrz * cam1) / a2) *
                    (2 * py *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * pz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) -
                            2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (ca - (rz2 * cam1) / a2) * (2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                               2 * px *
                                                       ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                               (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                               2 * py *
                                                       (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * rxrz2 * cam1) / a4 - (2 * rx * sa) / a + (2 * rxrz2 * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2)) +
            ((4 * rx2rz * cam1) / a4 - (2 * rz * cam1) / a2 + (2 * ca * rxry) / a2 - (2 * rxry * sa) / a3 +
                    (2 * rx2rz * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2));
    d2F_dzdx(0, 3) = 2 * px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4) -
                     2 * pz *
                             ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                     (rx2rz * sa) / a3) -
                     2 * py *
                             ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                     (rx2ry * sa) / a3);
    d2F_dzdx(0, 4) =
            -2 * py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) -
            2 * px *
                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                            (rx2ry * sa) / a3) -
            2 * pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(0, 5) =
            -2 * pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) -
            2 * px *
                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                            (rx2rz * sa) / a3) -
            2 * py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(1, 0) =
            ((2 * ry2 * sa) / a3 - (2 * ca * ry2) / a2 - (2 * sa) / a + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2)) -
            ((ry * sa) / a + (rxrz * cam1) / a2) *
                    (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) +
                            2 * px *
                                    ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            ((rz * sa) / a - (rxry * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            (ca - (rx2 * cam1) / a2) * (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                               2 * py *
                                                       ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                               (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                               2 * pz *
                                                       (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * rx2ry * cam1) / a4 - (2 * ry * sa) / a + (2 * rx2ry * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
            ((4 * rxry2 * cam1) / a4 - (2 * rx * cam1) / a2 + (2 * ca * ryrz) / a2 - (2 * ryrz * sa) / a3 +
                    (2 * rxry2 * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(1, 1) =
            ((rx * sa) / a - (ryrz * cam1) / a2) *
                    (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) +
                            2 * px *
                                    ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((rz * sa) / a + (rxry * cam1) / a2) *
                    (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                            2 * py *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * pz *
                                    (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((2 * ry * sa) / a + (4 * ry * cam1) / a2 - (2 * ry3 * sa) / a3 - (4 * ry3 * cam1) / a4) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
            (ca - (ry2 * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            ((4 * rxry2 * cam1) / a4 - (2 * rx * cam1) / a2 - (2 * ca * ryrz) / a2 + (2 * ryrz * sa) / a3 +
                    (2 * rxry2 * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
            ((4 * ry2rz * cam1) / a4 - (2 * rz * cam1) / a2 + (2 * ca * rxry) / a2 - (2 * rxry * sa) / a3 +
                    (2 * ry2rz * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(1, 2) =
            ((2 * sa) / a + (2 * ca * ry2) / a2 - (2 * ry2 * sa) / a3 + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
            ((ry * sa) / a - (rxrz * cam1) / a2) *
                    (2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                            2 * py *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * pz *
                                    (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((rx * sa) / a + (ryrz * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * pz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) -
                            2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            (ca - (rz2 * cam1) / a2) * (2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                               2 * py *
                                                       ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                               (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                               2 * px *
                                                       ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * ryrz2 * cam1) / a4 - (2 * ry * sa) / a + (2 * ryrz2 * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2)) +
            ((4 * ry2rz * cam1) / a4 - (2 * rz * cam1) / a2 - (2 * ca * rxry) / a2 + (2 * rxry * sa) / a3 +
                    (2 * ry2rz * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(1, 3) =
            -2 * px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) -
            2 * py *
                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                            (rxry2 * sa) / a3) -
            2 * pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(1, 4) = 2 * py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4) -
                     2 * pz *
                             ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                     (ry2rz * sa) / a3) -
                     2 * px *
                             ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                     (rxry2 * sa) / a3);
    d2F_dzdx(1, 5) =
            -2 * pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) -
            2 * py *
                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                            (ry2rz * sa) / a3) -
            2 * px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(2, 0) =
            ((2 * sa) / a + (2 * ca * rz2) / a2 - (2 * rz2 * sa) / a3 + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
            ((rz * sa) / a - (rxry * cam1) / a2) *
                    (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) +
                            2 * px *
                                    (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((ry * sa) / a + (rxrz * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) +
            (ca - (rx2 * cam1) / a2) * (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                               2 * pz *
                                                       ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                               (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                               2 * py *
                                                       ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * rx2rz * cam1) / a4 - (2 * rz * sa) / a + (2 * rx2rz * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
            ((4 * rxrz2 * cam1) / a4 - (2 * rx * cam1) / a2 - (2 * ca * ryrz) / a2 + (2 * ryrz * sa) / a3 +
                    (2 * rxrz2 * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(2, 1) =
            ((2 * rz2 * sa) / a3 - (2 * ca * rz2) / a2 - (2 * sa) / a + (4 * rxryrz * cam1) / a4 +
                    (2 * rxryrz * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) -
            ((rz * sa) / a + (rxry * cam1) / a2) *
                    (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            ((rx * sa) / a - (ryrz * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) +
            (ca - (ry2 * cam1) / a2) * (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                               2 * pz *
                                                       ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                               (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                               2 * px *
                                                       (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                               (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
            ((4 * ry2rz * cam1) / a4 - (2 * rz * sa) / a + (2 * ry2rz * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
            ((4 * ryrz2 * cam1) / a4 - (2 * ry * cam1) / a2 + (2 * ca * rxrz) / a2 - (2 * rxrz * sa) / a3 +
                    (2 * ryrz2 * sa) / a3) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(2, 2) =
            ((ry * sa) / a - (rxrz * cam1) / a2) *
                    (2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) -
            ((2 * rz * sa) / a + (4 * rz * cam1) / a2 - (2 * rz3 * sa) / a3 - (4 * rz3 * cam1) / a4) *
                    (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                            py * ((rx * sa) / a - (ryrz * cam1) / a2)) -
            ((rx * sa) / a + (ryrz * cam1) / a2) *
                    (2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            2 * pz *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) +
                            2 * px *
                                    (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (ca - (rz2 * cam1) / a2) *
                    (2 * px *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * py *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) -
                            2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) +
            ((4 * rxrz2 * cam1) / a4 - (2 * rx * cam1) / a2 + (2 * ca * ryrz) / a2 - (2 * ryrz * sa) / a3 +
                    (2 * rxrz2 * sa) / a3) *
                    (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                            pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
            ((4 * ryrz2 * cam1) / a4 - (2 * ry * cam1) / a2 - (2 * ca * rxrz) / a2 + (2 * rxrz * sa) / a3 +
                    (2 * ryrz2 * sa) / a3) *
                    (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                            pz * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(2, 3) =
            -2 * px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) -
            2 * pz *
                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                            (rxrz2 * sa) / a3) -
            2 * py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(2, 4) =
            -2 * py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) -
            2 * pz *
                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                            (ryrz2 * sa) / a3) -
            2 * px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3);
    d2F_dzdx(2, 5) = 2 * pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4) -
                     2 * py *
                             ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                     (ryrz2 * sa) / a3) -
                     2 * px *
                             ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                     (rxrz2 * sa) / a3);
    d2F_dzdx(3, 0) = 2 * ca - (2 * rx2 * cam1) / a2;
    d2F_dzdx(3, 1) = -(2 * rz * sa) / a - (2 * rxry * cam1) / a2;
    d2F_dzdx(3, 2) = (2 * ry * sa) / a - (2 * rxrz * cam1) / a2;
    d2F_dzdx(3, 3) = -2;
    d2F_dzdx(3, 4) = 0;
    d2F_dzdx(3, 5) = 0;
    d2F_dzdx(4, 0) = (2 * rz * sa) / a - (2 * rxry * cam1) / a2;
    d2F_dzdx(4, 1) = 2 * ca - (2 * ry2 * cam1) / a2;
    d2F_dzdx(4, 2) = -(2 * rx * sa) / a - (2 * ryrz * cam1) / a2;
    d2F_dzdx(4, 3) = 0;
    d2F_dzdx(4, 4) = -2;
    d2F_dzdx(4, 5) = 0;
    d2F_dzdx(5, 0) = -(2 * ry * sa) / a - (2 * rxrz * cam1) / a2;
    d2F_dzdx(5, 1) = (2 * rx * sa) / a - (2 * ryrz * cam1) / a2;
    d2F_dzdx(5, 2) = 2 * ca - (2 * rz2 * cam1) / a2;
    d2F_dzdx(5, 3) = 0;
    d2F_dzdx(5, 4) = 0;
    d2F_dzdx(5, 5) = -2;

    return d2F_dzdx;
}

template<typename PointSource, typename PointTarget, typename Scalar>
bool PointToPointIcpLinearised<PointSource, PointTarget, Scalar>::process_correspondence(
        const PointSource& source_point, const PointTarget& target_point,
        const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
        Eigen::Matrix<double, 6, 6>& d2F_dzdx) const {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    d2F_dx2 = compute_d2F_dx2(tf.r(), tf.t(), p, q);
    d2F_dzdx = compute_d2F_dzdx(tf.r(), tf.t(), p, q);
    return true;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPointIcpLinearised<PointSource, PointTarget, Scalar>::compute_d2F_dx2(
        const Eigen::Matrix<double, 3, 1>& r, const Eigen::Matrix<double, 3, 1>& t,
        const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q) const {
    // Aliases
    const double rx{r[0]};
    const double ry{r[1]};
    const double rz{r[2]};
    const double tx{t[0]};
    const double ty{t[1]};
    const double tz{t[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double px2 = px * px;
    const double py2 = py * py;
    const double pz2 = pz * pz;

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dx2;
    d2F_dx2(0, 0) = 2 * py2 + 2 * pz2;
    d2F_dx2(0, 1) = -2 * px * py;
    d2F_dx2(0, 2) = -2 * px * pz;
    d2F_dx2(0, 3) = 0;
    d2F_dx2(0, 4) = -2 * pz;
    d2F_dx2(0, 5) = 2 * py;
    d2F_dx2(1, 0) = -2 * px * py;
    d2F_dx2(1, 1) = 2 * px2 + 2 * pz2;
    d2F_dx2(1, 2) = -2 * py * pz;
    d2F_dx2(1, 3) = 2 * pz;
    d2F_dx2(1, 4) = 0;
    d2F_dx2(1, 5) = -2 * px;
    d2F_dx2(2, 0) = -2 * px * pz;
    d2F_dx2(2, 1) = -2 * py * pz;
    d2F_dx2(2, 2) = 2 * px2 + 2 * py2;
    d2F_dx2(2, 3) = -2 * py;
    d2F_dx2(2, 4) = 2 * px;
    d2F_dx2(2, 5) = 0;
    d2F_dx2(3, 0) = 0;
    d2F_dx2(3, 1) = 2 * pz;
    d2F_dx2(3, 2) = -2 * py;
    d2F_dx2(3, 3) = 2;
    d2F_dx2(3, 4) = 0;
    d2F_dx2(3, 5) = 0;
    d2F_dx2(4, 0) = -2 * pz;
    d2F_dx2(4, 1) = 0;
    d2F_dx2(4, 2) = 2 * px;
    d2F_dx2(4, 3) = 0;
    d2F_dx2(4, 4) = 2;
    d2F_dx2(4, 5) = 0;
    d2F_dx2(5, 0) = 2 * py;
    d2F_dx2(5, 1) = -2 * px;
    d2F_dx2(5, 2) = 0;
    d2F_dx2(5, 3) = 0;
    d2F_dx2(5, 4) = 0;
    d2F_dx2(5, 5) = 2;
    return d2F_dx2;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPointIcpLinearised<PointSource, PointTarget, Scalar>::compute_d2F_dzdx(
        const Eigen::Matrix<double, 3, 1>& r, const Eigen::Matrix<double, 3, 1>& t,
        const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q) const {
    // Aliases
    const double rx{r[0]};
    const double ry{r[1]};
    const double rz{r[2]};
    const double tx{t[0]};
    const double ty{t[1]};
    const double tz{t[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dzdx;
    d2F_dzdx(0, 0) = -2 * py * ry - 2 * pz * rz;
    d2F_dzdx(0, 1) = 2 * tz - 2 * qz - 2 * px * ry + 4 * py * rx;
    d2F_dzdx(0, 2) = 2 * qy - 2 * ty - 2 * px * rz + 4 * pz * rx;
    d2F_dzdx(0, 3) = 0;
    d2F_dzdx(0, 4) = 2 * pz;
    d2F_dzdx(0, 5) = -2 * py;
    d2F_dzdx(1, 0) = 2 * qz - 2 * tz + 4 * px * ry - 2 * py * rx;
    d2F_dzdx(1, 1) = -2 * px * rx - 2 * pz * rz;
    d2F_dzdx(1, 2) = 2 * tx - 2 * qx - 2 * py * rz + 4 * pz * ry;
    d2F_dzdx(1, 3) = -2 * pz;
    d2F_dzdx(1, 4) = 0;
    d2F_dzdx(1, 5) = 2 * px;
    d2F_dzdx(2, 0) = 2 * ty - 2 * qy + 4 * px * rz - 2 * pz * rx;
    d2F_dzdx(2, 1) = 2 * qx - 2 * tx + 4 * py * rz - 2 * pz * ry;
    d2F_dzdx(2, 2) = -2 * px * rx - 2 * py * ry;
    d2F_dzdx(2, 3) = 2 * py;
    d2F_dzdx(2, 4) = -2 * px;
    d2F_dzdx(2, 5) = 0;
    d2F_dzdx(3, 0) = 2;
    d2F_dzdx(3, 1) = -2 * rz;
    d2F_dzdx(3, 2) = 2 * ry;
    d2F_dzdx(3, 3) = -2;
    d2F_dzdx(3, 4) = 0;
    d2F_dzdx(3, 5) = 0;
    d2F_dzdx(4, 0) = 2 * rz;
    d2F_dzdx(4, 1) = 2;
    d2F_dzdx(4, 2) = -2 * rx;
    d2F_dzdx(4, 3) = 0;
    d2F_dzdx(4, 4) = -2;
    d2F_dzdx(4, 5) = 0;
    d2F_dzdx(5, 0) = -2 * ry;
    d2F_dzdx(5, 1) = 2 * rx;
    d2F_dzdx(5, 2) = 2;
    d2F_dzdx(5, 3) = 0;
    d2F_dzdx(5, 4) = 0;
    d2F_dzdx(5, 5) = -2;
    return d2F_dzdx;
}

template<typename PointSource, typename PointTarget, typename Scalar>
bool PointToPlaneIcpNonlinear<PointSource, PointTarget, Scalar>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& d2F_dx2, Eigen::Matrix<double, 6, 6>& d2F_dzdx) const {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    d2F_dx2 = compute_d2F_dx2(tf, p, q, n);
    d2F_dzdx = compute_d2F_dzdx(tf, p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpNonlinear<PointSource, PointTarget, Scalar>::compute_d2F_dx2(
        const PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) const {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx4 = rx * rx * rx * rx;      // rx4
    const double ry4 = ry * ry * ry * ry;      // ry4
    const double rz4 = rz * rz * rz * rz;      // rz4
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx2
    const double ry2 = ry * ry;                // ry2
    const double rz2 = rz * rz;                // rz2
    const double rxry = rx * ry;               // rxry
    const double ryrz = ry * rz;               // ryrz
    const double rxrz = rx * rz;               // rxrz
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double a5 = a4 * tf.a();             // a5
    const double a6 = a5 * tf.a();             // a6
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;   // cam1
    const double nx2 = nx * nx;   // nx2
    const double ny2 = ny * ny;   // ny2
    const double nz2 = nz * nz;   // nz2
    const double nxny = nx * ny;  // nxny
    const double nynz = ny * nz;  // nynz
    const double nxnz = nx * nz;  // nxnz

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dx2;
    d2F_dx2(0, 0) =
            ((2 * nx *
                             (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                           (rx2ry * sa) / a3) +
                                     pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                  (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                     px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                  (2 * rx3 * cam1) / a4)) +
                     (2 * ny *
                             (px * ry * a2 - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa - 2 * pz * rxryrz -
                                     a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * px * ry * a2 - ca * pz * rx2 * a2 - a * px * rxrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa +
                                     ca * px * rxrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * a2 - 2 * py * rxryrz -
                                     a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * px * rz * a2 + ca * py * rx2 * a2 + a * px * rxry * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa -
                                     ca * px * rxry * a2 + a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    a4 -
            (2 * nx *
                            (px * (sa / a + (2 * cam1) / a2 + (ca * rx2) / a2 - (ca * rx4) / a4 - (6 * rx2 * sa) / a3 +
                                          (5 * rx4 * sa) / a5 - (10 * rx2 * cam1) / a4 + (8 * rx4 * cam1) / a6) -
                                    py * ((rz * sa) / a3 - (ca * rz) / a2 - (8 * rx3ry * cam1) / a6 +
                                                 (3 * rxry * sa) / a3 + (6 * rxry * cam1) / a4 + (ca * rx3ry) / a4 +
                                                 (3 * ca * rx2rz) / a4 - (5 * rx3ry * sa) / a5 + (rx2rz * sa) / a3 -
                                                 (3 * rx2rz * sa) / a5) +
                                    pz * ((ry * sa) / a3 - (ca * ry) / a2 + (8 * rx3rz * cam1) / a6 -
                                                 (3 * rxrz * sa) / a3 + (3 * ca * rx2ry) / a4 - (6 * rxrz * cam1) / a4 -
                                                 (ca * rx3rz) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 +
                                                 (5 * rx3rz * sa) / a5)) +
                    2 * ny *
                            (py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 - (ry2 * sa) / a3 -
                                          (2 * ry2 * cam1) / a4 - (ca * rx2ry2) / a4 + (5 * rx2ry2 * sa) / a5 +
                                          (8 * rx2ry2 * cam1) / a6) +
                                    px * ((rz * sa) / a3 - (ca * rz) / a2 + (8 * rx3ry * cam1) / a6 -
                                                 (3 * rxry * sa) / a3 - (6 * rxry * cam1) / a4 - (ca * rx3ry) / a4 +
                                                 (3 * ca * rx2rz) / a4 + (5 * rx3ry * sa) / a5 + (rx2rz * sa) / a3 -
                                                 (3 * rx2rz * sa) / a5) -
                                    pz * ((3 * rx * sa) / a3 + (3 * ca * rx3) / a4 + (rx3 * sa) / a3 -
                                                 (3 * rx3 * sa) / a5 - (3 * ca * rx) / a2 + (ryrz * sa) / a3 +
                                                 (2 * ryrz * cam1) / a4 + (ca * rx2ryrz) / a4 -
                                                 (5 * rx2ryrz * sa) / a5 - (8 * rx2ryrz * cam1) / a6)) +
                    2 * nz *
                            (pz * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 - (rz2 * sa) / a3 -
                                          (2 * rz2 * cam1) / a4 - (ca * rx2rz2) / a4 + (5 * rx2rz2 * sa) / a5 +
                                          (8 * rx2rz2 * cam1) / a6) -
                                    px * ((ry * sa) / a3 - (ca * ry) / a2 - (8 * rx3rz * cam1) / a6 +
                                                 (3 * rxrz * sa) / a3 + (3 * ca * rx2ry) / a4 + (6 * rxrz * cam1) / a4 +
                                                 (ca * rx3rz) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 -
                                                 (5 * rx3rz * sa) / a5) +
                                    py * ((3 * rx * sa) / a3 + (3 * ca * rx3) / a4 + (rx3 * sa) / a3 -
                                                 (3 * rx3 * sa) / a5 - (3 * ca * rx) / a2 - (ryrz * sa) / a3 -
                                                 (2 * ryrz * cam1) / a4 - (ca * rx2ryrz) / a4 +
                                                 (5 * rx2ryrz * sa) / a5 + (8 * rx2ryrz * cam1) / a6))) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2)));
    d2F_dx2(0, 1) =
            (2 * ny *
                            (px * ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                          (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                          (8 * rx2ry2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    py * ((8 * rxry3 * cam1) / a6 + (ca * rxry) / a2 - (3 * rxry * sa) / a3 -
                                                 (4 * rxry * cam1) / a4 - (ca * rxry3) / a4 + (5 * rxry3 * sa) / a5) +
                                    pz * ((ry * sa) / a3 - (ca * ry) / a2 + (rxrz * sa) / a3 + (3 * ca * rx2ry) / a4 +
                                                 (2 * rxrz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 +
                                                 (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 -
                                                 (8 * rxry2rz * cam1) / a6)) -
                    2 * nx *
                            (px * ((8 * rx3ry * cam1) / a6 + (ca * rxry) / a2 - (3 * rxry * sa) / a3 -
                                          (4 * rxry * cam1) / a4 - (ca * rx3ry) / a4 + (5 * rx3ry * sa) / a5) -
                                    py * ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                                 (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                                 (8 * rx2ry2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    pz * ((rx * sa) / a3 - (ca * rx) / a2 - (ryrz * sa) / a3 + (3 * ca * rxry2) / a4 -
                                                 (2 * ryrz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 -
                                                 (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 +
                                                 (8 * rx2ryrz * cam1) / a6)) +
                    (2 * nz *
                            (a4 * ca * py * ry - a4 * ca * px * rx + a3 * px * rx * sa - a3 * py * ry * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * px * ryrz * a2 -
                                    2 * py * rxrz * a2 + a3 * px * rxry2 * sa - a3 * py * rx2ry * sa +
                                    3 * ca * px * rxry2 * a2 - 3 * ca * py * rx2ry * a2 - a4 * ca * pz * rxry -
                                    3 * a * px * rxry2 * sa + 3 * a * py * rx2ry * sa + a3 * px * ryrz * sa +
                                    a3 * py * rxrz * sa + a3 * pz * rxry * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * px * ryrz * a2 +
                                    2 * ca * py * rxrz * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * nx *
                             (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                           (rx2ry * sa) / a3) +
                                     pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                  (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                     px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                  (2 * rx3 * cam1) / a4)) +
                     (2 * ny *
                             (px * ry * a2 - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa - 2 * pz * rxryrz -
                                     a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * px * ry * a2 - ca * pz * rx2 * a2 - a * px * rxrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa +
                                     ca * px * rxrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * a2 - 2 * py * rxryrz -
                                     a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * px * rz * a2 + ca * py * rx2 * a2 + a * px * rxry * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa -
                                     ca * px * rxry * a2 + a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    a4;
    d2F_dx2(0, 2) =
            (2 * nx *
                            (pz * ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                          (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                          (8 * rx2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    px * ((8 * rx3rz * cam1) / a6 + (ca * rxrz) / a2 - (3 * rxrz * sa) / a3 -
                                                 (4 * rxrz * cam1) / a4 - (ca * rx3rz) / a4 + (5 * rx3rz * sa) / a5) +
                                    py * ((rx * sa) / a3 - (ca * rx) / a2 + (ryrz * sa) / a3 + (3 * ca * rxrz2) / a4 +
                                                 (2 * ryrz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 +
                                                 (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 -
                                                 (8 * rx2ryrz * cam1) / a6)) -
                    2 * nz *
                            (pz * ((8 * rxrz3 * cam1) / a6 + (ca * rxrz) / a2 - (3 * rxrz * sa) / a3 -
                                          (4 * rxrz * cam1) / a4 - (ca * rxrz3) / a4 + (5 * rxrz3 * sa) / a5) -
                                    px * ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                                 (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                                 (8 * rx2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    py * ((rz * sa) / a3 - (ca * rz) / a2 - (rxry * sa) / a3 - (2 * rxry * cam1) / a4 +
                                                 (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 -
                                                 (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 +
                                                 (8 * rxryrz2 * cam1) / a6)) +
                    (2 * ny *
                            (a4 * ca * px * rx - a4 * ca * pz * rz - a3 * px * rx * sa + a3 * pz * rz * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * px * ryrz * a2 -
                                    2 * pz * rxry * a2 - a3 * px * rxrz2 * sa + a3 * pz * rx2rz * sa -
                                    3 * ca * px * rxrz2 * a2 + 3 * ca * pz * rx2rz * a2 - a4 * ca * py * rxrz +
                                    3 * a * px * rxrz2 * sa + a3 * px * ryrz * sa + a3 * py * rxrz * sa +
                                    a3 * pz * rxry * sa - 3 * a * pz * rx2rz * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * px * ryrz * a2 +
                                    2 * ca * pz * rxry * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * nx *
                             (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                           (rx2ry * sa) / a3) +
                                     pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                  (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                     px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                  (2 * rx3 * cam1) / a4)) +
                     (2 * ny *
                             (px * ry * a2 - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa - 2 * pz * rxryrz -
                                     a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * px * ry * a2 - ca * pz * rx2 * a2 - a * px * rxrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa +
                                     ca * px * rxrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * a2 - 2 * py * rxryrz -
                                     a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * px * rz * a2 + ca * py * rx2 * a2 + a * px * rxry * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa -
                                     ca * px * rxry * a2 + a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    a4;
    d2F_dx2(0, 3) = nx * (2 * ny *
                                         (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                 px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                 px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                 py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nx *
                                         (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                       (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                 px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                              (2 * rx3 * cam1) / a4)));
    d2F_dx2(0, 4) = ny * (2 * ny *
                                         (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                 px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                 px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                 py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nx *
                                         (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                       (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                 px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                              (2 * rx3 * cam1) / a4)));
    d2F_dx2(0, 5) = nz * (2 * ny *
                                         (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                 px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                 px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                 py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nx *
                                         (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                       (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                 pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                 px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                              (2 * rx3 * cam1) / a4)));
    d2F_dx2(1, 0) =
            (2 * ny *
                            (px * ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                          (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                          (8 * rx2ry2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    py * ((8 * rxry3 * cam1) / a6 + (ca * rxry) / a2 - (3 * rxry * sa) / a3 -
                                                 (4 * rxry * cam1) / a4 - (ca * rxry3) / a4 + (5 * rxry3 * sa) / a5) +
                                    pz * ((ry * sa) / a3 - (ca * ry) / a2 + (rxrz * sa) / a3 + (3 * ca * rx2ry) / a4 +
                                                 (2 * rxrz * cam1) / a4 + (rx2ry * sa) / a3 - (3 * rx2ry * sa) / a5 +
                                                 (ca * rxry2rz) / a4 - (5 * rxry2rz * sa) / a5 -
                                                 (8 * rxry2rz * cam1) / a6)) -
                    2 * nx *
                            (px * ((8 * rx3ry * cam1) / a6 + (ca * rxry) / a2 - (3 * rxry * sa) / a3 -
                                          (4 * rxry * cam1) / a4 - (ca * rx3ry) / a4 + (5 * rx3ry * sa) / a5) -
                                    py * ((rx2 * sa) / a3 - cam1 / a2 + (ry2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                                 (2 * ry2 * cam1) / a4 + (ca * rx2ry2) / a4 - (5 * rx2ry2 * sa) / a5 -
                                                 (8 * rx2ry2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    pz * ((rx * sa) / a3 - (ca * rx) / a2 - (ryrz * sa) / a3 + (3 * ca * rxry2) / a4 -
                                                 (2 * ryrz * cam1) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 -
                                                 (ca * rx2ryrz) / a4 + (5 * rx2ryrz * sa) / a5 +
                                                 (8 * rx2ryrz * cam1) / a6)) +
                    (2 * nz *
                            (a4 * ca * py * ry - a4 * ca * px * rx + a3 * px * rx * sa - a3 * py * ry * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * px * ryrz * a2 -
                                    2 * py * rxrz * a2 + a3 * px * rxry2 * sa - a3 * py * rx2ry * sa +
                                    3 * ca * px * rxry2 * a2 - 3 * ca * py * rx2ry * a2 - a4 * ca * pz * rxry -
                                    3 * a * px * rxry2 * sa + 3 * a * py * rx2ry * sa + a3 * px * ryrz * sa +
                                    a3 * py * rxrz * sa + a3 * pz * rxry * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * px * ryrz * a2 +
                                    2 * ca * py * rxrz * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * ny *
                             (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                           (rxry2 * sa) / a3) +
                                     pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                  (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                     py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                  (2 * ry3 * cam1) / a4)) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * a2 - 2 * pz * rxryrz -
                                     a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * py * rx * a2 + ca * pz * ry2 * a2 + a * py * ryrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa -
                                     ca * py * ryrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * a2 - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa - 2 * px * rxryrz +
                                     a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * py * rz * a2 - ca * px * ry2 * a2 - a * py * rxry * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa +
                                     ca * py * rxry * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    a4;
    d2F_dx2(1, 1) =
            ((2 * ny *
                             (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                           (rxry2 * sa) / a3) +
                                     pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                  (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                     py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                  (2 * ry3 * cam1) / a4)) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * a2 - 2 * pz * rxryrz -
                                     a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * py * rx * a2 + ca * pz * ry2 * a2 + a * py * ryrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa -
                                     ca * py * ryrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * a2 - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa - 2 * px * rxryrz +
                                     a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * py * rz * a2 - ca * px * ry2 * a2 - a * py * rxry * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa +
                                     ca * py * rxry * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    a4 -
            (2 * ny *
                            (py * (sa / a + (2 * cam1) / a2 + (ca * ry2) / a2 - (ca * ry4) / a4 - (6 * ry2 * sa) / a3 +
                                          (5 * ry4 * sa) / a5 - (10 * ry2 * cam1) / a4 + (8 * ry4 * cam1) / a6) +
                                    px * ((rz * sa) / a3 - (ca * rz) / a2 + (8 * rxry3 * cam1) / a6 -
                                                 (3 * rxry * sa) / a3 - (6 * rxry * cam1) / a4 - (ca * rxry3) / a4 +
                                                 (3 * ca * ry2rz) / a4 + (5 * rxry3 * sa) / a5 + (ry2rz * sa) / a3 -
                                                 (3 * ry2rz * sa) / a5) -
                                    pz * ((rx * sa) / a3 - (ca * rx) / a2 - (8 * ry3rz * cam1) / a6 +
                                                 (3 * ryrz * sa) / a3 + (3 * ca * rxry2) / a4 + (6 * ryrz * cam1) / a4 +
                                                 (ca * ry3rz) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 -
                                                 (5 * ry3rz * sa) / a5)) +
                    2 * nx *
                            (px * (sa / a + (ca * ry2) / a2 - (rx2 * sa) / a3 - (ry2 * sa) / a3 -
                                          (2 * rx2 * cam1) / a4 - (ca * rx2ry2) / a4 + (5 * rx2ry2 * sa) / a5 +
                                          (8 * rx2ry2 * cam1) / a6) -
                                    py * ((rz * sa) / a3 - (ca * rz) / a2 - (8 * rxry3 * cam1) / a6 +
                                                 (3 * rxry * sa) / a3 + (6 * rxry * cam1) / a4 + (ca * rxry3) / a4 +
                                                 (3 * ca * ry2rz) / a4 - (5 * rxry3 * sa) / a5 + (ry2rz * sa) / a3 -
                                                 (3 * ry2rz * sa) / a5) +
                                    pz * ((3 * ry * sa) / a3 + (3 * ca * ry3) / a4 + (ry3 * sa) / a3 -
                                                 (3 * ry3 * sa) / a5 - (3 * ca * ry) / a2 - (rxrz * sa) / a3 -
                                                 (2 * rxrz * cam1) / a4 - (ca * rxry2rz) / a4 +
                                                 (5 * rxry2rz * sa) / a5 + (8 * rxry2rz * cam1) / a6)) +
                    2 * nz *
                            (pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 - (rz2 * sa) / a3 -
                                          (2 * rz2 * cam1) / a4 - (ca * ry2rz2) / a4 + (5 * ry2rz2 * sa) / a5 +
                                          (8 * ry2rz2 * cam1) / a6) -
                                    px * ((3 * ry * sa) / a3 + (3 * ca * ry3) / a4 + (ry3 * sa) / a3 -
                                                 (3 * ry3 * sa) / a5 - (3 * ca * ry) / a2 + (rxrz * sa) / a3 +
                                                 (2 * rxrz * cam1) / a4 + (ca * rxry2rz) / a4 -
                                                 (5 * rxry2rz * sa) / a5 - (8 * rxry2rz * cam1) / a6) +
                                    py * ((rx * sa) / a3 - (ca * rx) / a2 + (8 * ry3rz * cam1) / a6 -
                                                 (3 * ryrz * sa) / a3 + (3 * ca * rxry2) / a4 - (6 * ryrz * cam1) / a4 -
                                                 (ca * ry3rz) / a4 + (rxry2 * sa) / a3 - (3 * rxry2 * sa) / a5 +
                                                 (5 * ry3rz * sa) / a5))) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2)));
    d2F_dx2(1, 2) =
            (2 * nz *
                            (py * ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                          (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                          (8 * ry2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    pz * ((8 * ryrz3 * cam1) / a6 + (ca * ryrz) / a2 - (3 * ryrz * sa) / a3 -
                                                 (4 * ryrz * cam1) / a4 - (ca * ryrz3) / a4 + (5 * ryrz3 * sa) / a5) +
                                    px * ((rz * sa) / a3 - (ca * rz) / a2 + (rxry * sa) / a3 + (2 * rxry * cam1) / a4 +
                                                 (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 +
                                                 (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 -
                                                 (8 * rxryrz2 * cam1) / a6)) -
                    2 * ny *
                            (py * ((8 * ry3rz * cam1) / a6 + (ca * ryrz) / a2 - (3 * ryrz * sa) / a3 -
                                          (4 * ryrz * cam1) / a4 - (ca * ry3rz) / a4 + (5 * ry3rz * sa) / a5) -
                                    pz * ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                                 (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                                 (8 * ry2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    px * ((ry * sa) / a3 - (ca * ry) / a2 - (rxrz * sa) / a3 - (2 * rxrz * cam1) / a4 +
                                                 (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 -
                                                 (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 +
                                                 (8 * rxry2rz * cam1) / a6)) +
                    (2 * nx *
                            (a4 * ca * pz * rz - a4 * ca * py * ry + a3 * py * ry * sa - a3 * pz * rz * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * py * rxrz * a2 -
                                    2 * pz * rxry * a2 + a3 * py * ryrz2 * sa - a3 * pz * ry2rz * sa +
                                    3 * ca * py * ryrz2 * a2 - 3 * ca * pz * ry2rz * a2 - a4 * ca * px * ryrz +
                                    a3 * px * ryrz * sa + a3 * py * rxrz * sa + a3 * pz * rxry * sa -
                                    3 * a * py * ryrz2 * sa + 3 * a * pz * ry2rz * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * py * rxrz * a2 +
                                    2 * ca * pz * rxry * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * ny *
                             (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                           (rxry2 * sa) / a3) +
                                     pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                  (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                     py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                  (2 * ry3 * cam1) / a4)) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * a2 - 2 * pz * rxryrz -
                                     a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry + 2 * ca * py * rxry2 -
                                     ca * py * rx * a2 + ca * pz * ry2 * a2 + a * py * ryrz * sa +
                                     2 * ca * pz * rxryrz + a * px * rx2ry * sa + a * py * rxry2 * sa -
                                     ca * py * ryrz * a2 + a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * a2 - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa - 2 * px * rxryrz +
                                     a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * py * rz * a2 - ca * px * ry2 * a2 - a * py * rxry * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa +
                                     ca * py * rxry * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    a4;
    d2F_dx2(1, 3) = nx * (2 * nx *
                                         (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                 py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                              (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                 py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                 px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                       (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                 py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                              (2 * ry3 * cam1) / a4)));
    d2F_dx2(1, 4) = ny * (2 * nx *
                                         (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                 py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                              (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                 py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                 px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                       (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                 py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                              (2 * ry3 * cam1) / a4)));
    d2F_dx2(1, 5) = nz * (2 * nx *
                                         (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                 py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                              (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                 py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                 px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                       (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                 pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                              (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                 py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                              (2 * ry3 * cam1) / a4)));
    d2F_dx2(2, 0) =
            (2 * nx *
                            (pz * ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                          (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                          (8 * rx2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    px * ((8 * rx3rz * cam1) / a6 + (ca * rxrz) / a2 - (3 * rxrz * sa) / a3 -
                                                 (4 * rxrz * cam1) / a4 - (ca * rx3rz) / a4 + (5 * rx3rz * sa) / a5) +
                                    py * ((rx * sa) / a3 - (ca * rx) / a2 + (ryrz * sa) / a3 + (3 * ca * rxrz2) / a4 +
                                                 (2 * ryrz * cam1) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 +
                                                 (ca * rx2ryrz) / a4 - (5 * rx2ryrz * sa) / a5 -
                                                 (8 * rx2ryrz * cam1) / a6)) -
                    2 * nz *
                            (pz * ((8 * rxrz3 * cam1) / a6 + (ca * rxrz) / a2 - (3 * rxrz * sa) / a3 -
                                          (4 * rxrz * cam1) / a4 - (ca * rxrz3) / a4 + (5 * rxrz3 * sa) / a5) -
                                    px * ((rx2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * rx2 * cam1) / a4 +
                                                 (2 * rz2 * cam1) / a4 + (ca * rx2rz2) / a4 - (5 * rx2rz2 * sa) / a5 -
                                                 (8 * rx2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    py * ((rz * sa) / a3 - (ca * rz) / a2 - (rxry * sa) / a3 - (2 * rxry * cam1) / a4 +
                                                 (3 * ca * rx2rz) / a4 + (rx2rz * sa) / a3 - (3 * rx2rz * sa) / a5 -
                                                 (ca * rxryrz2) / a4 + (5 * rxryrz2 * sa) / a5 +
                                                 (8 * rxryrz2 * cam1) / a6)) +
                    (2 * ny *
                            (a4 * ca * px * rx - a4 * ca * pz * rz - a3 * px * rx * sa + a3 * pz * rz * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * px * ryrz * a2 -
                                    2 * pz * rxry * a2 - a3 * px * rxrz2 * sa + a3 * pz * rx2rz * sa -
                                    3 * ca * px * rxrz2 * a2 + 3 * ca * pz * rx2rz * a2 - a4 * ca * py * rxrz +
                                    3 * a * px * rxrz2 * sa + a3 * px * ryrz * sa + a3 * py * rxrz * sa +
                                    a3 * pz * rxry * sa - 3 * a * pz * rx2rz * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * px * ryrz * a2 +
                                    2 * ca * pz * rxry * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * nz *
                             (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                           (rxrz2 * sa) / a3) +
                                     py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                  (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                     pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                  (2 * rz3 * cam1) / a4)) +
                     (2 * nx *
                             (pz * rx * a2 - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa - 2 * py * rxryrz -
                                     a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * pz * rx * a2 - ca * py * rz2 * a2 - a * pz * ryrz * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa +
                                     ca * pz * ryrz * a2 + a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * a2 - 2 * px * rxryrz -
                                     a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * pz * ry * a2 + ca * px * rz2 * a2 + a * pz * rxrz * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa -
                                     ca * pz * rxrz * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    a4;
    d2F_dx2(2, 1) =
            (2 * nz *
                            (py * ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                          (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                          (8 * ry2rz2 * cam1) / a6 - (3 * ca * rxryrz) / a4 - (rxryrz * sa) / a3 +
                                          (3 * rxryrz * sa) / a5) -
                                    pz * ((8 * ryrz3 * cam1) / a6 + (ca * ryrz) / a2 - (3 * ryrz * sa) / a3 -
                                                 (4 * ryrz * cam1) / a4 - (ca * ryrz3) / a4 + (5 * ryrz3 * sa) / a5) +
                                    px * ((rz * sa) / a3 - (ca * rz) / a2 + (rxry * sa) / a3 + (2 * rxry * cam1) / a4 +
                                                 (3 * ca * ry2rz) / a4 + (ry2rz * sa) / a3 - (3 * ry2rz * sa) / a5 +
                                                 (ca * rxryrz2) / a4 - (5 * rxryrz2 * sa) / a5 -
                                                 (8 * rxryrz2 * cam1) / a6)) -
                    2 * ny *
                            (py * ((8 * ry3rz * cam1) / a6 + (ca * ryrz) / a2 - (3 * ryrz * sa) / a3 -
                                          (4 * ryrz * cam1) / a4 - (ca * ry3rz) / a4 + (5 * ry3rz * sa) / a5) -
                                    pz * ((ry2 * sa) / a3 - cam1 / a2 + (rz2 * sa) / a3 + (2 * ry2 * cam1) / a4 +
                                                 (2 * rz2 * cam1) / a4 + (ca * ry2rz2) / a4 - (5 * ry2rz2 * sa) / a5 -
                                                 (8 * ry2rz2 * cam1) / a6 + (3 * ca * rxryrz) / a4 +
                                                 (rxryrz * sa) / a3 - (3 * rxryrz * sa) / a5) +
                                    px * ((ry * sa) / a3 - (ca * ry) / a2 - (rxrz * sa) / a3 - (2 * rxrz * cam1) / a4 +
                                                 (3 * ca * ryrz2) / a4 + (ryrz2 * sa) / a3 - (3 * ryrz2 * sa) / a5 -
                                                 (ca * rxry2rz) / a4 + (5 * rxry2rz * sa) / a5 +
                                                 (8 * rxry2rz * cam1) / a6)) +
                    (2 * nx *
                            (a4 * ca * pz * rz - a4 * ca * py * ry + a3 * py * ry * sa - a3 * pz * rz * sa +
                                    8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 - 2 * py * rxrz * a2 -
                                    2 * pz * rxry * a2 + a3 * py * ryrz2 * sa - a3 * pz * ry2rz * sa +
                                    3 * ca * py * ryrz2 * a2 - 3 * ca * pz * ry2rz * a2 - a4 * ca * px * ryrz +
                                    a3 * px * ryrz * sa + a3 * py * rxrz * sa + a3 * pz * rxry * sa -
                                    3 * a * py * ryrz2 * sa + 3 * a * pz * ry2rz * sa - 8 * ca * px * rx2ryrz -
                                    8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 + 2 * ca * py * rxrz * a2 +
                                    2 * ca * pz * rxry * a2 - 5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa -
                                    5 * a * pz * rxryrz2 * sa + ca * px * rx2ryrz * a2 + ca * py * rxry2rz * a2 +
                                    ca * pz * rxryrz2 * a2)) /
                            a6) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2))) +
            ((2 * nz *
                             (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                           (rxrz2 * sa) / a3) +
                                     py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                  (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                     pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                  (2 * rz3 * cam1) / a4)) +
                     (2 * nx *
                             (pz * rx * a2 - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa - 2 * py * rxryrz -
                                     a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * pz * rx * a2 - ca * py * rz2 * a2 - a * pz * ryrz * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa +
                                     ca * pz * ryrz * a2 + a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * a2 - 2 * px * rxryrz -
                                     a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * pz * ry * a2 + ca * px * rz2 * a2 + a * pz * rxrz * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa -
                                     ca * pz * rxrz * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    a4;
    d2F_dx2(2, 2) =
            ((2 * nz *
                             (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                           (rxrz2 * sa) / a3) +
                                     py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                  (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                     pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                  (2 * rz3 * cam1) / a4)) +
                     (2 * nx *
                             (pz * rx * a2 - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa - 2 * py * rxryrz -
                                     a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz + 2 * ca * pz * rxrz2 -
                                     ca * pz * rx * a2 - ca * py * rz2 * a2 - a * pz * ryrz * sa +
                                     2 * ca * py * rxryrz + a * px * rx2rz * sa + a * pz * rxrz2 * sa +
                                     ca * pz * ryrz * a2 + a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * a2 - 2 * px * rxryrz -
                                     a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz + 2 * ca * pz * ryrz2 -
                                     ca * pz * ry * a2 + ca * px * rz2 * a2 + a * pz * rxrz * sa +
                                     2 * ca * px * rxryrz + a * py * ry2rz * sa + a * pz * ryrz2 * sa -
                                     ca * pz * rxrz * a2 + a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    a4 -
            (2 * nz *
                            (pz * (sa / a + (2 * cam1) / a2 + (ca * rz2) / a2 - (ca * rz4) / a4 - (6 * rz2 * sa) / a3 +
                                          (5 * rz4 * sa) / a5 - (10 * rz2 * cam1) / a4 + (8 * rz4 * cam1) / a6) -
                                    px * ((ry * sa) / a3 - (ca * ry) / a2 - (8 * rxrz3 * cam1) / a6 +
                                                 (3 * rxrz * sa) / a3 + (6 * rxrz * cam1) / a4 + (ca * rxrz3) / a4 +
                                                 (3 * ca * ryrz2) / a4 - (5 * rxrz3 * sa) / a5 + (ryrz2 * sa) / a3 -
                                                 (3 * ryrz2 * sa) / a5) +
                                    py * ((rx * sa) / a3 - (ca * rx) / a2 + (8 * ryrz3 * cam1) / a6 -
                                                 (3 * ryrz * sa) / a3 + (3 * ca * rxrz2) / a4 - (6 * ryrz * cam1) / a4 -
                                                 (ca * ryrz3) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 +
                                                 (5 * ryrz3 * sa) / a5)) +
                    2 * nx *
                            (px * (sa / a + (ca * rz2) / a2 - (rx2 * sa) / a3 - (rz2 * sa) / a3 -
                                          (2 * rx2 * cam1) / a4 - (ca * rx2rz2) / a4 + (5 * rx2rz2 * sa) / a5 +
                                          (8 * rx2rz2 * cam1) / a6) -
                                    py * ((3 * rz * sa) / a3 + (3 * ca * rz3) / a4 + (rz3 * sa) / a3 -
                                                 (3 * rz3 * sa) / a5 - (3 * ca * rz) / a2 + (rxry * sa) / a3 +
                                                 (2 * rxry * cam1) / a4 + (ca * rxryrz2) / a4 -
                                                 (5 * rxryrz2 * sa) / a5 - (8 * rxryrz2 * cam1) / a6) +
                                    pz * ((ry * sa) / a3 - (ca * ry) / a2 + (8 * rxrz3 * cam1) / a6 -
                                                 (3 * rxrz * sa) / a3 - (6 * rxrz * cam1) / a4 - (ca * rxrz3) / a4 +
                                                 (3 * ca * ryrz2) / a4 + (5 * rxrz3 * sa) / a5 + (ryrz2 * sa) / a3 -
                                                 (3 * ryrz2 * sa) / a5)) +
                    2 * ny *
                            (py * (sa / a + (ca * rz2) / a2 - (ry2 * sa) / a3 - (rz2 * sa) / a3 -
                                          (2 * ry2 * cam1) / a4 - (ca * ry2rz2) / a4 + (5 * ry2rz2 * sa) / a5 +
                                          (8 * ry2rz2 * cam1) / a6) +
                                    px * ((3 * rz * sa) / a3 + (3 * ca * rz3) / a4 + (rz3 * sa) / a3 -
                                                 (3 * rz3 * sa) / a5 - (3 * ca * rz) / a2 - (rxry * sa) / a3 -
                                                 (2 * rxry * cam1) / a4 - (ca * rxryrz2) / a4 +
                                                 (5 * rxryrz2 * sa) / a5 + (8 * rxryrz2 * cam1) / a6) -
                                    pz * ((rx * sa) / a3 - (ca * rx) / a2 - (8 * ryrz3 * cam1) / a6 +
                                                 (3 * ryrz * sa) / a3 + (3 * ca * rxrz2) / a4 + (6 * ryrz * cam1) / a4 +
                                                 (ca * ryrz3) / a4 + (rxrz2 * sa) / a3 - (3 * rxrz2 * sa) / a5 -
                                                 (5 * ryrz3 * sa) / a5))) *
                    (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                                  pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                            ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                         pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                            nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                         py * ((rx * sa) / a - (ryrz * cam1) / a2)));
    d2F_dx2(2, 3) = nx * (2 * nx *
                                         (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                 pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                              (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                 pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                 px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                       (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                 pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                              (2 * rz3 * cam1) / a4)));
    d2F_dx2(2, 4) = ny * (2 * nx *
                                         (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                 pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                              (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                 pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                 px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                       (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                 pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                              (2 * rz3 * cam1) / a4)));
    d2F_dx2(2, 5) = nz * (2 * nx *
                                         (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                 pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                              (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * ny *
                                         (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                 pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                 px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                              (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                 2 * nz *
                                         (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                       (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                 py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                              (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                 pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                              (2 * rz3 * cam1) / a4)));
    d2F_dx2(3, 0) =
            (2 * nx *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    d2F_dx2(3, 1) =
            (2 * nx *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    d2F_dx2(3, 2) =
            (2 * nx *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    d2F_dx2(3, 3) = 2 * nx2;
    d2F_dx2(3, 4) = 2 * nxny;
    d2F_dx2(3, 5) = 2 * nxnz;
    d2F_dx2(4, 0) =
            (2 * ny *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    d2F_dx2(4, 1) =
            (2 * ny *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    d2F_dx2(4, 2) =
            (2 * ny *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    d2F_dx2(4, 3) = 2 * nxny;
    d2F_dx2(4, 4) = 2 * ny2;
    d2F_dx2(4, 5) = 2 * nynz;
    d2F_dx2(5, 0) =
            (2 * nz *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * a2 + nx * py * ry * a2 +
                            ny * px * ry * a2 + nx * pz * rz * a2 + nz * px * rz * a2 - ca * ny * pz * rx2 * a2 +
                            ca * nz * py * rx2 * a2 - 2 * ny * pz * rxryrz - 2 * nz * py * rxryrz +
                            a * nx * px * rx3 * sa - a3 * nx * px * rx * sa - a3 * ny * py * rx * sa +
                            a * ny * pz * rx2 * sa - a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa +
                            2 * ca * nx * py * rx2ry + 2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 +
                            2 * ca * nx * pz * rx2rz + 2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * a2 - ca * nx * py * ry * a2 - ca * ny * px * ry * a2 -
                            ca * nx * pz * rz * a2 - ca * nz * px * rz * a2 + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * a2 +
                            ca * nx * pz * rxry * a2 + ca * ny * px * rxrz * a2 - ca * nz * px * rxry * a2 +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    d2F_dx2(5, 1) =
            (2 * nz *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * a2 + ny * px * rx * a2 +
                            2 * ny * py * ry * a2 + ny * pz * rz * a2 + nz * py * rz * a2 + ca * nx * pz * ry2 * a2 -
                            ca * nz * px * ry2 * a2 - 2 * nx * pz * rxryrz - 2 * nz * px * rxryrz -
                            a3 * nx * px * ry * sa - a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa +
                            a * ny * py * ry3 * sa - a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa +
                            2 * ca * nx * px * rx2ry + 2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 +
                            2 * ca * ny * pz * ry2rz + 2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * a2 - ca * ny * px * rx * a2 - 2 * ca * ny * py * ry * a2 -
                            ca * ny * pz * rz * a2 - ca * nz * py * rz * a2 + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa - ca * nx * py * ryrz * a2 +
                            ca * ny * px * ryrz * a2 - ca * ny * pz * rxry * a2 + ca * nz * py * rxry * a2 +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    d2F_dx2(5, 2) =
            (2 * nz *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * a2 + nz * px * rx * a2 +
                            ny * pz * ry * a2 + nz * py * ry * a2 + 2 * nz * pz * rz * a2 - ca * nx * py * rz2 * a2 +
                            ca * ny * px * rz2 * a2 - 2 * nx * py * rxryrz - 2 * ny * px * rxryrz -
                            a3 * nx * px * rz * sa + a * nx * py * rz2 * sa - a * ny * px * rz2 * sa -
                            a3 * ny * py * rz * sa + a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa +
                            2 * ca * nx * px * rx2rz + 2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 +
                            2 * ca * ny * py * ry2rz + 2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * a2 - ca * nz * px * rx * a2 - ca * ny * pz * ry * a2 -
                            ca * nz * py * ry * a2 - 2 * ca * nz * pz * rz * a2 + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa + ca * nx * pz * ryrz * a2 -
                            ca * ny * pz * rxrz * a2 - ca * nz * px * ryrz * a2 + ca * nz * py * rxrz * a2 -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    d2F_dx2(5, 3) = 2 * nxnz;
    d2F_dx2(5, 4) = 2 * nynz;
    d2F_dx2(5, 5) = 2 * nz2;
    return d2F_dx2;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpNonlinear<PointSource, PointTarget, Scalar>::compute_d2F_dzdx(
        const PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) const {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx4 = rx * rx * rx * rx;      // rx4
    const double ry4 = ry * ry * ry * ry;      // ry4
    const double rz4 = rz * rz * rz * rz;      // rz4
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx2
    const double ry2 = ry * ry;                // ry2
    const double rz2 = rz * rz;                // rz2
    const double rxry = rx * ry;               // rxry
    const double ryrz = ry * rz;               // ryrz
    const double rxrz = rx * rz;               // rxrz
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;   // cam1
    const double nx2 = nx * nx;   // nx2
    const double ny2 = ny * ny;   // ny2
    const double nz2 = nz * nz;   // nz2
    const double nxny = nx * ny;  // nxny
    const double nynz = ny * nz;  // nynz
    const double nxnz = nx * nz;  // nxnz

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dzdx;
    d2F_dzdx(0, 0) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * ny *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * nz *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) -
                            2 * nx * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 - (2 * rx3 * cam1) / a4)) +
            (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                    nz * ((ry * sa) / a + (rxrz * cam1) / a2)) *
                    (2 * ny *
                                    (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                            px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                            px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                            py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nx *
                                    (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                  (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                            px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                         (2 * rx3 * cam1) / a4)));
    d2F_dzdx(0, 1) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * ny * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                            2 * nx *
                                    ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (rx2ry * sa) / a3) +
                            2 * nz *
                                    (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                    nz * ((rx * sa) / a - (ryrz * cam1) / a2)) *
                    (2 * ny *
                                    (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                            px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                            px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                            py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nx *
                                    (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                  (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                            px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                         (2 * rx3 * cam1) / a4)));
    d2F_dzdx(0, 2) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                            2 * nx *
                                    ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (rx2rz * sa) / a3) +
                            2 * ny *
                                    ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                    ny * ((rx * sa) / a + (ryrz * cam1) / a2)) *
                    (2 * ny *
                                    (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                            px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                            px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                            py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nx *
                                    (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                  (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                            pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                            px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                         (2 * rx3 * cam1) / a4)));
    d2F_dzdx(0, 3) = -nx * (2 * ny *
                                           (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                   px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                   px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                   py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nx *
                                           (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                   px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                                (2 * rx3 * cam1) / a4)));
    d2F_dzdx(0, 4) = -ny * (2 * ny *
                                           (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                   px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                   px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                   py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nx *
                                           (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                   px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                                (2 * rx3 * cam1) / a4)));
    d2F_dzdx(0, 5) = -nz * (2 * ny *
                                           (py * ((2 * rxry2 * cam1) / a4 - (rx * sa) / a + (rxry2 * sa) / a3) +
                                                   px * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((rx2 * sa) / a3 - (ca * rx2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * rxrz2 * cam1) / a4 - (rx * sa) / a + (rxrz2 * sa) / a3) +
                                                   px * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) +
                                                   py * (sa / a + (ca * rx2) / a2 - (rx2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nx *
                                           (py * ((2 * rx2ry * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (rx2ry * sa) / a3) +
                                                   pz * ((2 * rx2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (rx2rz * sa) / a3) -
                                                   px * ((rx * sa) / a + (2 * rx * cam1) / a2 - (rx3 * sa) / a3 -
                                                                (2 * rx3 * cam1) / a4)));
    d2F_dzdx(1, 0) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nx * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                            2 * ny *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * nz *
                                    ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                    nz * ((ry * sa) / a + (rxrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                            py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                            py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                            px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                  (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                            py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                         (2 * ry3 * cam1) / a4)));
    d2F_dzdx(1, 1) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nx *
                                    ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxry2 * sa) / a3) +
                            2 * nz *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 - (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) -
                            2 * ny * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 - (2 * ry3 * cam1) / a4)) +
            (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                    nz * ((rx * sa) / a - (ryrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                            py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                            py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                            px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                  (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                            py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                         (2 * ry3 * cam1) / a4)));
    d2F_dzdx(1, 2) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                            2 * ny *
                                    ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 + (rxry * sa) / a3 +
                                            (ry2rz * sa) / a3) +
                            2 * nx *
                                    (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                    ny * ((rx * sa) / a + (ryrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                            py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                            py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                            px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                  (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                            pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                         (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                            py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                         (2 * ry3 * cam1) / a4)));
    d2F_dzdx(1, 3) = -nx * (2 * nx *
                                           (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                   py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                                (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                   py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                   px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                   py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                                (2 * ry3 * cam1) / a4)));
    d2F_dzdx(1, 4) = -ny * (2 * nx *
                                           (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                   py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                                (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                   py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                   px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                   py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                                (2 * ry3 * cam1) / a4)));
    d2F_dzdx(1, 5) = -nz * (2 * nx *
                                           (px * ((2 * rx2ry * cam1) / a4 - (ry * sa) / a + (rx2ry * sa) / a3) +
                                                   py * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                                (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * (sa / a + (ca * ry2) / a2 - (ry2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (pz * ((2 * ryrz2 * cam1) / a4 - (ry * sa) / a + (ryrz2 * sa) / a3) +
                                                   py * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 + (ca * rxry) / a2 -
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) +
                                                   px * ((ry2 * sa) / a3 - (ca * ry2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (px * ((2 * rxry2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxry2 * sa) / a3) +
                                                   pz * ((2 * ry2rz * cam1) / a4 - (rz * cam1) / a2 - (ca * rxry) / a2 +
                                                                (rxry * sa) / a3 + (ry2rz * sa) / a3) -
                                                   py * ((ry * sa) / a + (2 * ry * cam1) / a2 - (ry3 * sa) / a3 -
                                                                (2 * ry3 * cam1) / a4)));
    d2F_dzdx(2, 0) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nx * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                            2 * nz *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 + (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * ny *
                                    (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                    nz * ((ry * sa) / a + (rxrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                            pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                            pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                            px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                  (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                            pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                         (2 * rz3 * cam1) / a4)));
    d2F_dzdx(2, 1) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * ny * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                            2 * nz *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 - (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) +
                            2 * nx *
                                    ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a + (2 * rxryrz * cam1) / a4 +
                                            (rxryrz * sa) / a3)) +
            (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                    nz * ((rx * sa) / a - (ryrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                            pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                            pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                            px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                  (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                            pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                         (2 * rz3 * cam1) / a4)));
    d2F_dzdx(2, 2) =
            (nx * (tx - qx + px * (ca - (rx2 * cam1) / a2) - py * ((rz * sa) / a + (rxry * cam1) / a2) +
                          pz * ((ry * sa) / a - (rxrz * cam1) / a2)) +
                    ny * (ty - qy + py * (ca - (ry2 * cam1) / a2) + px * ((rz * sa) / a - (rxry * cam1) / a2) -
                                 pz * ((rx * sa) / a + (ryrz * cam1) / a2)) +
                    nz * (tz - qz + pz * (ca - (rz2 * cam1) / a2) - px * ((ry * sa) / a + (rxrz * cam1) / a2) +
                                 py * ((rx * sa) / a - (ryrz * cam1) / a2))) *
                    (2 * nx *
                                    ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 - (ryrz * sa) / a3 +
                                            (rxrz2 * sa) / a3) +
                            2 * ny *
                                    ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 + (rxrz * sa) / a3 +
                                            (ryrz2 * sa) / a3) -
                            2 * nz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 - (2 * rz3 * cam1) / a4)) +
            (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                    ny * ((rx * sa) / a + (ryrz * cam1) / a2)) *
                    (2 * nx *
                                    (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                            pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * ny *
                                    (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                            pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                            px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                         (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                            2 * nz *
                                    (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                  (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                            py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                         (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                            pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                         (2 * rz3 * cam1) / a4)));
    d2F_dzdx(2, 3) = -nx * (2 * nx *
                                           (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                   pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                                (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                   pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                   px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                   pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                                (2 * rz3 * cam1) / a4)));
    d2F_dzdx(2, 4) = -ny * (2 * nx *
                                           (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                   pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                                (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                   pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                   px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                   pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                                (2 * rz3 * cam1) / a4)));
    d2F_dzdx(2, 5) = -nz * (2 * nx *
                                           (px * ((2 * rx2rz * cam1) / a4 - (rz * sa) / a + (rx2rz * sa) / a3) +
                                                   pz * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 + (ca * ryrz) / a2 -
                                                                (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((rz2 * sa) / a3 - (ca * rz2) / a2 - sa / a +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * ny *
                                           (py * ((2 * ry2rz * cam1) / a4 - (rz * sa) / a + (ry2rz * sa) / a3) +
                                                   pz * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 - (ca * rxrz) / a2 +
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) +
                                                   px * (sa / a + (ca * rz2) / a2 - (rz2 * sa) / a3 +
                                                                (2 * rxryrz * cam1) / a4 + (rxryrz * sa) / a3)) +
                                   2 * nz *
                                           (px * ((2 * rxrz2 * cam1) / a4 - (rx * cam1) / a2 - (ca * ryrz) / a2 +
                                                         (ryrz * sa) / a3 + (rxrz2 * sa) / a3) +
                                                   py * ((2 * ryrz2 * cam1) / a4 - (ry * cam1) / a2 + (ca * rxrz) / a2 -
                                                                (rxrz * sa) / a3 + (ryrz2 * sa) / a3) -
                                                   pz * ((rz * sa) / a + (2 * rz * cam1) / a2 - (rz3 * sa) / a3 -
                                                                (2 * rz3 * cam1) / a4)));
    d2F_dzdx(3, 0) = 2 * nx *
                     (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                             nz * ((ry * sa) / a + (rxrz * cam1) / a2));
    d2F_dzdx(3, 1) = 2 * nx *
                     (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                             nz * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(3, 2) = 2 * nx *
                     (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                             ny * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(3, 3) = -2 * nx2;
    d2F_dzdx(3, 4) = -2 * nxny;
    d2F_dzdx(3, 5) = -2 * nxnz;
    d2F_dzdx(4, 0) = 2 * ny *
                     (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                             nz * ((ry * sa) / a + (rxrz * cam1) / a2));
    d2F_dzdx(4, 1) = 2 * ny *
                     (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                             nz * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(4, 2) = 2 * ny *
                     (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                             ny * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(4, 3) = -2 * nxny;
    d2F_dzdx(4, 4) = -2 * ny2;
    d2F_dzdx(4, 5) = -2 * nynz;
    d2F_dzdx(5, 0) = 2 * nz *
                     (nx * (ca - (rx2 * cam1) / a2) + ny * ((rz * sa) / a - (rxry * cam1) / a2) -
                             nz * ((ry * sa) / a + (rxrz * cam1) / a2));
    d2F_dzdx(5, 1) = 2 * nz *
                     (ny * (ca - (ry2 * cam1) / a2) - nx * ((rz * sa) / a + (rxry * cam1) / a2) +
                             nz * ((rx * sa) / a - (ryrz * cam1) / a2));
    d2F_dzdx(5, 2) = 2 * nz *
                     (nz * (ca - (rz2 * cam1) / a2) + nx * ((ry * sa) / a - (rxrz * cam1) / a2) -
                             ny * ((rx * sa) / a + (ryrz * cam1) / a2));
    d2F_dzdx(5, 3) = -2 * nxnz;
    d2F_dzdx(5, 4) = -2 * nynz;
    d2F_dzdx(5, 5) = -2 * nz2;
    return d2F_dzdx;
}

template<typename PointSource, typename PointTarget, typename Scalar>
bool PointToPlaneIcpLinearised<PointSource, PointTarget, Scalar>::process_correspondence(
        const PointSource& source_point, const PointTarget& target_point,
        const PrecomputedTransformComponents<double>& tf, Eigen::Matrix<double, 6, 6>& d2F_dx2,
        Eigen::Matrix<double, 6, 6>& d2F_dzdx) const {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    d2F_dx2 = compute_d2F_dx2(tf.r(), tf.t(), p, q, n);
    d2F_dzdx = compute_d2F_dzdx(tf.r(), tf.t(), p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpLinearised<PointSource, PointTarget, Scalar>::compute_d2F_dx2(
        const Eigen::Matrix<double, 3, 1>& r, const Eigen::Matrix<double, 3, 1>& t,
        const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
        const Eigen::Matrix<double, 3, 1>& n) const {
    // Aliases
    const double rx{r[0]};
    const double ry{r[1]};
    const double rz{r[2]};
    const double tx{t[0]};
    const double ty{t[1]};
    const double tz{t[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double nx2 = nx * nx;
    const double ny2 = ny * ny;
    const double nz2 = nz * nz;

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dx2;
    d2F_dx2(0, 0) = (ny * pz - nz * py) * (2 * ny * pz - 2 * nz * py);
    d2F_dx2(0, 1) = -(nx * pz - nz * px) * (2 * ny * pz - 2 * nz * py);
    d2F_dx2(0, 2) = (nx * py - ny * px) * (2 * ny * pz - 2 * nz * py);
    d2F_dx2(0, 3) = -2 * nx * (ny * pz - nz * py);
    d2F_dx2(0, 4) = 2 * ny * nz * py - 2 * ny2 * pz;
    d2F_dx2(0, 5) = 2 * nz2 * py - 2 * ny * nz * pz;
    d2F_dx2(1, 0) = -(2 * nx * pz - 2 * nz * px) * (ny * pz - nz * py);
    d2F_dx2(1, 1) = (nx * pz - nz * px) * (2 * nx * pz - 2 * nz * px);
    d2F_dx2(1, 2) = -(nx * py - ny * px) * (2 * nx * pz - 2 * nz * px);
    d2F_dx2(1, 3) = nx * (2 * nx * pz - 2 * nz * px);
    d2F_dx2(1, 4) = ny * (2 * nx * pz - 2 * nz * px);
    d2F_dx2(1, 5) = nz * (2 * nx * pz - 2 * nz * px);
    d2F_dx2(2, 0) = (2 * nx * py - 2 * ny * px) * (ny * pz - nz * py);
    d2F_dx2(2, 1) = -(2 * nx * py - 2 * ny * px) * (nx * pz - nz * px);
    d2F_dx2(2, 2) = (nx * py - ny * px) * (2 * nx * py - 2 * ny * px);
    d2F_dx2(2, 3) = 2 * nx * ny * px - 2 * nx2 * py;
    d2F_dx2(2, 4) = 2 * ny2 * px - 2 * nx * ny * py;
    d2F_dx2(2, 5) = -2 * nz * (nx * py - ny * px);
    d2F_dx2(3, 0) = -2 * nx * (ny * pz - nz * py);
    d2F_dx2(3, 1) = 2 * nx * (nx * pz - nz * px);
    d2F_dx2(3, 2) = -2 * nx * (nx * py - ny * px);
    d2F_dx2(3, 3) = 2 * nx2;
    d2F_dx2(3, 4) = 2 * nx * ny;
    d2F_dx2(3, 5) = 2 * nx * nz;
    d2F_dx2(4, 0) = -2 * ny * (ny * pz - nz * py);
    d2F_dx2(4, 1) = 2 * ny * (nx * pz - nz * px);
    d2F_dx2(4, 2) = -2 * ny * (nx * py - ny * px);
    d2F_dx2(4, 3) = 2 * nx * ny;
    d2F_dx2(4, 4) = 2 * ny2;
    d2F_dx2(4, 5) = 2 * ny * nz;
    d2F_dx2(5, 0) = -2 * nz * (ny * pz - nz * py);
    d2F_dx2(5, 1) = 2 * nz * (nx * pz - nz * px);
    d2F_dx2(5, 2) = -2 * nz * (nx * py - ny * px);
    d2F_dx2(5, 3) = 2 * nx * nz;
    d2F_dx2(5, 4) = 2 * ny * nz;
    d2F_dx2(5, 5) = 2 * nz2;
    return d2F_dx2;
}

template<typename PointSource, typename PointTarget, typename Scalar>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpLinearised<PointSource, PointTarget, Scalar>::compute_d2F_dzdx(
        const Eigen::Matrix<double, 3, 1>& r, const Eigen::Matrix<double, 3, 1>& t,
        const Eigen::Matrix<double, 3, 1>& p, const Eigen::Matrix<double, 3, 1>& q,
        const Eigen::Matrix<double, 3, 1>& n) const {
    // Aliases
    const double rx{r[0]};
    const double ry{r[1]};
    const double rz{r[2]};
    const double tx{t[0]};
    const double ty{t[1]};
    const double tz{t[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double nx2 = nx * nx;
    const double ny2 = ny * ny;
    const double nz2 = nz * nz;

    // Fill elements
    Eigen::Matrix<double, 6, 6> d2F_dzdx;
    d2F_dzdx(0, 0) = -(2 * ny * pz - 2 * nz * py) * (nx + ny * rz - nz * ry);
    d2F_dzdx(0, 1) = 2 * nz *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx)) -
                     (2 * ny * pz - 2 * nz * py) * (ny - nx * rz + nz * rx);
    d2F_dzdx(0, 2) = -2 * ny *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx)) -
                     (2 * ny * pz - 2 * nz * py) * (nz + nx * ry - ny * rx);
    d2F_dzdx(0, 3) = nx * (2 * ny * pz - 2 * nz * py);
    d2F_dzdx(0, 4) = ny * (2 * ny * pz - 2 * nz * py);
    d2F_dzdx(0, 5) = nz * (2 * ny * pz - 2 * nz * py);
    d2F_dzdx(1, 0) = (2 * nx * pz - 2 * nz * px) * (nx + ny * rz - nz * ry) -
                     2 * nz *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx));
    d2F_dzdx(1, 1) = (2 * nx * pz - 2 * nz * px) * (ny - nx * rz + nz * rx);
    d2F_dzdx(1, 2) = 2 * nx *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx)) +
                     (2 * nx * pz - 2 * nz * px) * (nz + nx * ry - ny * rx);
    d2F_dzdx(1, 3) = 2 * nx * nz * px - 2 * nx2 * pz;
    d2F_dzdx(1, 4) = -2 * ny * (nx * pz - nz * px);
    d2F_dzdx(1, 5) = 2 * nz2 * px - 2 * nx * nz * pz;
    d2F_dzdx(2, 0) = 2 * ny *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx)) -
                     (2 * nx * py - 2 * ny * px) * (nx + ny * rz - nz * ry);
    d2F_dzdx(2, 1) = -2 * nx *
                             (nx * (px - qx + tx - py * rz + pz * ry) + ny * (py - qy + ty + px * rz - pz * rx) +
                                     nz * (pz - qz + tz - px * ry + py * rx)) -
                     (2 * nx * py - 2 * ny * px) * (ny - nx * rz + nz * rx);
    d2F_dzdx(2, 2) = -(2 * nx * py - 2 * ny * px) * (nz + nx * ry - ny * rx);
    d2F_dzdx(2, 3) = nx * (2 * nx * py - 2 * ny * px);
    d2F_dzdx(2, 4) = ny * (2 * nx * py - 2 * ny * px);
    d2F_dzdx(2, 5) = nz * (2 * nx * py - 2 * ny * px);
    d2F_dzdx(3, 0) = 2 * nx * (nx + ny * rz - nz * ry);
    d2F_dzdx(3, 1) = 2 * nx * (ny - nx * rz + nz * rx);
    d2F_dzdx(3, 2) = 2 * nx * (nz + nx * ry - ny * rx);
    d2F_dzdx(3, 3) = -2 * nx2;
    d2F_dzdx(3, 4) = -2 * nx * ny;
    d2F_dzdx(3, 5) = -2 * nx * nz;
    d2F_dzdx(4, 0) = 2 * ny * (nx + ny * rz - nz * ry);
    d2F_dzdx(4, 1) = 2 * ny * (ny - nx * rz + nz * rx);
    d2F_dzdx(4, 2) = 2 * ny * (nz + nx * ry - ny * rx);
    d2F_dzdx(4, 3) = -2 * nx * ny;
    d2F_dzdx(4, 4) = -2 * ny2;
    d2F_dzdx(4, 5) = -2 * ny * nz;
    d2F_dzdx(5, 0) = 2 * nz * (nx + ny * rz - nz * ry);
    d2F_dzdx(5, 1) = 2 * nz * (ny - nx * rz + nz * rx);
    d2F_dzdx(5, 2) = 2 * nz * (nz + nx * ry - ny * rx);
    d2F_dzdx(5, 3) = -2 * nx * nz;
    d2F_dzdx(5, 4) = -2 * ny * nz;
    d2F_dzdx(5, 5) = -2 * nz2;
    return d2F_dzdx;
}

}

#endif
