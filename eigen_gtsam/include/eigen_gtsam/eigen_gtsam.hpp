#ifndef EIGEN_GTSAM_EIGEN_GTSAM_HPP
#define EIGEN_GTSAM_EIGEN_GTSAM_HPP

#include "eigen_gtsam/geometry.hpp"

namespace eigen_gtsam {

template<typename GTSAMType, typename EigenType>
GTSAMType to_gtsam(const EigenType& from);

template<typename EigenType, typename GTSAMType>
EigenType to_eigen(const GTSAMType& from);

/* Implementation */

template<typename GTSAMType, typename EigenType>
GTSAMType to_gtsam(const EigenType& from) {
    GTSAMType to;
    to_gtsam(from, to);
    return to;
}

template<typename EigenType, typename GTSAMType>
EigenType to_eigen(const GTSAMType& from) {
    EigenType to;
    to_eigen(from, to);
    return to;
}

}

#endif
