#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_ext {

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

/* Implementation */

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

}
