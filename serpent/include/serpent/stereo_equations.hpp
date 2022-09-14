#ifndef SERPENT_STEREO_EQUATIONS_HPP
#define SERPENT_STEREO_EQUATIONS_HPP

#include <Eigen/Core>

namespace serpent {

/**
 * @brief Backproject a stereo feature, defined by left, right and vertical coordinates, to 3D Euclidean space in the
 * left camera reference frame. This function assumes a basic stereo camera, with no vertical displacement between the
 * focal centres, that the image planes are co-planar, and that their centres are separated by a baseline distance.
 *
 * @tparam Scalar
 * @param stereo_coordinate [u_L, u_R, v]^T = vector of left horizontal coordinate, right horizontal coordinate,
 * vertical coordinate
 * @param intrinsic Stereo camera intrisic matrix (only focal lengths and principal point coordinates used)
 * @param baseline Stereo camera baseline distance
 * @return Eigen::Matrix<Scalar, 3, 1> 3D Euclidean point in left stereo camera reference frame
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> stereo_backproject(const Eigen::Matrix<Scalar, 3, 1>& stereo_coordinate,
        const Eigen::Matrix<Scalar, 3, 3>& intrinsic, const Scalar baseline);

/**
 * @brief Project a 3D Euclidean point to the image planes of a basic stereo camera, returning the stereo coordinates in
 * the form of left, right and vertical coordinates. The function assumes no vertical displacement between the stereo
 * image planes, and that they are separated by a horizontal baseline distance.
 *
 * @tparam Scalar
 * @param point [x, y, z] in the same units as baseline
 * @param intrinsic Stereo camera intrinsic matrix (only focal lengths and principal point coordinates used)
 * @param baseline Stereo camera baseline distance (same units as point)
 * @return Eigen::Matrix<Scalar, 3, 1>
 */
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> stereo_project(const Eigen::Matrix<Scalar, 3, 1>& point,
        const Eigen::Matrix<Scalar, 3, 3>& intrinsic, const Scalar baseline);

/* Implementation */

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> stereo_backproject(const Eigen::Matrix<Scalar, 3, 1>& stereo_coordinate,
        const Eigen::Matrix<Scalar, 3, 3>& intrinsic, const Scalar baseline) {
    const Scalar u_diff = stereo_coordinate[0] - stereo_coordinate[1];
    if (u_diff == 0.0) {
        throw std::runtime_error("Stereo horizontal coordinates are equal, point is infinitely far.");
    }
    return baseline *
           Eigen::Matrix<Scalar, 3, 1>{stereo_coordinate[0] - intrinsic(0, 2),
                   intrinsic(0, 0) * (stereo_coordinate[2] - intrinsic(1, 2)) / intrinsic(1, 1), intrinsic(0, 0)} /
           u_diff;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> stereo_project(const Eigen::Matrix<Scalar, 3, 1>& point,
        const Eigen::Matrix<Scalar, 3, 3>& intrinsic, const Scalar baseline) {
    if (point[3] <= 0.0) {
        throw std::runtime_error("Point is on or behind the image plane - cannot project onto plane.");
    }
    return Eigen::Matrix<Scalar, 3, 1>{intrinsic(0, 2), intrinsic(0, 2), intrinsic(1, 2)} +
           Eigen::Matrix<Scalar, 3, 1>{intrinsic(0, 0) * point[0], intrinsic(0, 0) * (point[0] - baseline),
                   intrinsic(1, 1) * point[1]} /
                   point[3];
}

}

#endif
