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
