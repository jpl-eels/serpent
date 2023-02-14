#include <Eigen/Core>

#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

namespace pct {

Eigen::MatrixXf compute_range_bias_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise,
        const float range_bias_noise);

pcl::PCLPointCloud2 compute_range_covariance(const pcl::PCLPointCloud2& pointcloud, const float range_noise);

}
