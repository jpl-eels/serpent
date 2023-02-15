#include <pointcloud_tools/pclpointcloud_transform.hpp>

namespace serpent {

template<typename PointT, typename Scalar>
void transform_pointcloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out,
        const Eigen::Matrix<Scalar, 4, 4>& transform) {
    pcl::transformPointCloudWithNormals(cloud_in, cloud_out, transform);
}

template<typename Scalar>
void transform_pointcloud(const pcl::PointCloud<PointNormalCovariance>& cloud_in,
        pcl::PointCloud<PointNormalCovariance>& cloud_out, const Eigen::Matrix<Scalar, 4, 4>& transform) {
    pct::transform_pointcloud_with_normals_and_covariances(cloud_in, cloud_out, transform);
}

}
