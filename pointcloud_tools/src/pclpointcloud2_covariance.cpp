#include "pointcloud_tools/pclpointcloud2_covariance.hpp"

namespace pct {

Eigen::MatrixXf compute_range_bias_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise,
        const float range_bias_noise) {
    try {
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
    } catch (const std::exception& ex) {
        throw std::runtime_error("Error thrown while computing covariance in compute_range_bias_covariance. Are there "
                                 "zero-length vectors? Does the pointcloud have x, y, z fields? Exception details: " +
                                 std::string(ex.what()));
    }
}

pcl::PCLPointCloud2 compute_range_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise) {
    try {
        pcl::PCLPointCloud2 new_pointcloud = pct::add_fields(pointcloud,
                {"covariance_xx", "covariance_xy", "covariance_xz", "covariance_yy", "covariance_yz", "covariance_zz"},
                pcl::PCLPointField::PointFieldTypes::FLOAT32);
        const auto& xx = get_field(new_pointcloud, "covariance_xx");
        const auto& xy = get_field(new_pointcloud, "covariance_xy");
        const auto& xz = get_field(new_pointcloud, "covariance_xz");
        const auto& yy = get_field(new_pointcloud, "covariance_yy");
        const auto& yz = get_field(new_pointcloud, "covariance_yz");
        const auto& zz = get_field(new_pointcloud, "covariance_zz");
        Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors = pct::unit_vectors(new_pointcloud);
        const float range_variance = range_noise * range_noise;
        for (std::size_t i = 0, j = 0; i < new_pointcloud.data.size(); i += new_pointcloud.point_step, ++j) {
            const Eigen::Matrix3f covariance = range_variance * unit_vectors.col(j) * unit_vectors.col(j).transpose();
            *reinterpret_cast<float*>(&new_pointcloud.data[i + xx.offset]) = covariance(0, 0);
            *reinterpret_cast<float*>(&new_pointcloud.data[i + xy.offset]) = covariance(0, 1);
            *reinterpret_cast<float*>(&new_pointcloud.data[i + xz.offset]) = covariance(0, 2);
            *reinterpret_cast<float*>(&new_pointcloud.data[i + yy.offset]) = covariance(1, 1);
            *reinterpret_cast<float*>(&new_pointcloud.data[i + yz.offset]) = covariance(1, 2);
            *reinterpret_cast<float*>(&new_pointcloud.data[i + zz.offset]) = covariance(2, 2);
        }
        return new_pointcloud;
    } catch (const std::exception& ex) {
        throw std::runtime_error("Error thrown while computing covariance in compute_range_covariance. Are there "
                                 "zero-length vectors? Does the pointcloud have x, y, z fields? Exception details: " +
                                 std::string(ex.what()));
    }
}

}
