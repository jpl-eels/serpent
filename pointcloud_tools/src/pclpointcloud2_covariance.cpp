#include "pointcloud_tools/pclpointcloud2_covariance.hpp"

namespace pct {

Eigen::MatrixXf compute_range_bias_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise,
        const float range_bias_noise) {
    const Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors = pct::unit_vectors(pointcloud);
    // Reshape as a column vector. Note from Eigen 3.4 onwards we can use unit_vectors.reshape().
    const Eigen::VectorXf unit_vectors_stacked =
            Eigen::Map<const Eigen::VectorXf>(unit_vectors.data(), unit_vectors.size());
    Eigen::MatrixXf covariance = unit_vectors_stacked * unit_vectors_stacked.transpose();
    const float range_variance = range_noise * range_noise;
    const float range_bias_variance = range_bias_noise * range_bias_noise;
    const float variance_sum = range_variance + range_bias_variance;
    // Iterate over blocks of 3
    for (std::size_t r = 0; r < covariance.rows(); r += 3) {
        for (std::size_t c = 0; c < covariance.cols(); c += 3) {
            covariance.block(r, c, 3, 3) *= (r == c ? variance_sum : range_bias_variance);
        }
    }
    return covariance;
}

pcl::PCLPointCloud2 compute_range_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise) {
    pcl::PCLPointCloud2 new_pointcloud =
            pct::add_field(pointcloud, "covariance", pcl::PCLPointField::PointFieldTypes::FLOAT32, 6);
    const auto& field = get_field(new_pointcloud, "covariance");
    const Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors = pct::unit_vectors(new_pointcloud);
    const float range_variance = range_noise * range_noise;
    for (std::size_t i = 0, j = 0; i < new_pointcloud.data.size(); i += new_pointcloud.point_step, ++j) {
        const Eigen::Matrix3f covariance = range_variance * unit_vectors.col(i) * unit_vectors.col(i).transpose();
        float* cov_data_ptr = reinterpret_cast<float*>(&new_pointcloud.data[i + field.offset]);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = r; c < 3; ++c) {
                *cov_data_ptr = covariance(r, c);
                ++cov_data_ptr;
            }
        }
    }
    return new_pointcloud;
}

}
