#ifndef POINTCLOUD_TOOLS_REGISTRATION_UTILITIES_HPP
#define POINTCLOUD_TOOLS_REGISTRATION_UTILITIES_HPP

#include <pcl/correspondence.h>
#include <pcl/registration/registration.h>

namespace pct {

/**
 * @brief Get correspondences for a completed registration. Only correspondences with distances <= max correspondence
 * distance will be returned. This is needed because pcl::Registration does not expose the correspondences to the user.
 *
 * @tparam PointSource
 * @tparam PointTarget
 * @tparam Scalar
 * @param registration
 * @return pcl::CorrespondencesPtr
 */
template<typename PointSource, typename PointTarget, typename Scalar>
pcl::CorrespondencesPtr compute_registration_correspondences(
        pcl::Registration<PointSource, PointTarget, Scalar>& registration,
        const bool use_reciprocal_correspondences = false);

}

#include "pointcloud_tools/impl/registration_utilities.hpp"

#endif
