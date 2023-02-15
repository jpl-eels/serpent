#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <gtest/gtest.h>

#include "eigen_ext/covariance.hpp"
#include "test_instances.hpp"

#define N (100)

TEST(unit_vectors, float_empty) {
    for (unsigned int i = 0; i < N; ++i) {
        Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors =
                pct::unit_vectors<float>(test_empty_pointcloud2<pcl::PointXYZ>(i));
        EXPECT_EQ(unit_vectors.rows(), 3);
        EXPECT_EQ(unit_vectors.cols(), 0);
    }
}

TEST(unit_vectors, float_points) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_x(static_cast<float>(i)));
        pointcloud.push_back(test_y(static_cast<float>(i)));
        pointcloud.push_back(test_z(static_cast<float>(i)));
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        if (i == 0) {
            EXPECT_THROW(pct::unit_vectors<float>(pointcloud2), std::runtime_error);
        } else {
            Eigen::Matrix<float, 3, Eigen::Dynamic> unit_vectors = pct::unit_vectors<float>(pointcloud2);
            EXPECT_EQ(unit_vectors.rows(), 3);
            EXPECT_EQ(unit_vectors.cols(), 4);
            EXPECT_EQ(unit_vectors.col(0), Eigen::Vector3f({1.f, 0.f, 0.f}));
            EXPECT_EQ(unit_vectors.col(1), Eigen::Vector3f({0.f, 1.f, 0.f}));
            EXPECT_EQ(unit_vectors.col(2), Eigen::Vector3f({0.f, 0.f, 1.f}));
            EXPECT_EQ(unit_vectors.col(3), Eigen::Vector3f({1.f, 1.f, 1.f}).normalized());
        }
    }
}
