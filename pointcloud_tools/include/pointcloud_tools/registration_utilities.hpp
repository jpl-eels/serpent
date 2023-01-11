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

/** Implementation */

// Note not using the radiusSearch function (with max_nn = 1) because it is not clear from documentation if it returns
// the first or the closest NN within the radius.
template<typename PointSource, typename PointTarget, typename Scalar>
pcl::CorrespondencesPtr compute_registration_correspondences(
        pcl::Registration<PointSource, PointTarget, Scalar>& registration,
        const bool use_reciprocal_correspondences) {
    pcl::CorrespondencesPtr correspondences = boost::make_shared<pcl::Correspondences>();
    const auto target_cloud = registration.getInputTarget();
    const double max_sq_dist = std::pow(registration.getMaxCorrespondenceDistance(), 2.0);
    const Eigen::Matrix<float, 4, 4> transform = registration.getFinalTransformation().template cast<float>();
    if (use_reciprocal_correspondences) {
        const std::size_t target_size = registration.getInputTarget()->size();
        const auto search = registration.getSearchMethodSource();
        if (search->getIndices() != registration.getIndices()) {
            throw std::runtime_error("kdtree search should be restricted to indices in reciprocal case");
        }
        pcl::Indices index(1);
        std::vector<float> sq_dist(1);
        for (std::size_t i = 0; i < target_size; ++i) {
            PointTarget target_point;
            // Search for transformed point: p_{S} = T_{S}^{T} * p_{T}
            target_point.getVector4fMap() = transform.inverse() * (*target_cloud)[i].getVector4fMap();
            if (search->nearestKSearch(target_point, 1, index, sq_dist) == 1 && sq_dist[0] <= max_sq_dist) {
                correspondences->emplace_back(index[0], i, std::sqrt(sq_dist[0]));
            }
        }
    } else {
        const std::size_t source_size = registration.getIndices()->size();  // Use indices size
        const auto search = registration.getSearchMethodTarget();
        pcl::Indices index(1);
        std::vector<float> sq_dist(1);
        for (std::size_t i = 0; i < source_size; ++i) {
            PointSource source_point;
            // Search for transformed point: p_{T} = T_{T}^{S} * p_{S}
            source_point.getVector4fMap() = transform * registration[i].getVector4fMap();  // Source point from indices
            if (search->nearestKSearch(source_point, 1, index, sq_dist) == 1 && sq_dist[0] <= max_sq_dist) {
                correspondences->emplace_back(i, index[0], std::sqrt(sq_dist[0]));
            }
        }
    }
    return correspondences;
}

}

#endif
