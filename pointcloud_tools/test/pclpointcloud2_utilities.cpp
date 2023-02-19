#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <gtest/gtest.h>

#include "eigen_ext/covariance.hpp"
#include "test_instances.hpp"

#define N (100)
#define FLOAT_PRECISION (1.e-7f)

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
            EXPECT_TRUE(unit_vectors.col(0).isApprox(Eigen::Vector3f({1.f, 0.f, 0.f})));
            EXPECT_TRUE(unit_vectors.col(1).isApprox(Eigen::Vector3f({0.f, 1.f, 0.f})));
            EXPECT_TRUE(unit_vectors.col(2).isApprox(Eigen::Vector3f({0.f, 0.f, 1.f})));
            EXPECT_TRUE(unit_vectors.col(3).isApprox(Eigen::Vector3f({1.f, 1.f, 1.f}).normalized()));
        }
    }
}

TEST(add_unit_vectors, float_points) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_x(static_cast<float>(i)));
        pointcloud.push_back(test_y(static_cast<float>(i)));
        pointcloud.push_back(test_z(static_cast<float>(i)));
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        if (i == 0) {
            EXPECT_THROW(pct::add_unit_vectors(pointcloud2), std::runtime_error);
        } else {
            pcl::PCLPointCloud2 pointcloud2_with_unit_vectors = pct::add_unit_vectors(pointcloud2);
            const pcl::PCLPointField& ux_field = pct::get_field(pointcloud2_with_unit_vectors, "ux");
            const pcl::PCLPointField& uy_field = pct::get_field(pointcloud2_with_unit_vectors, "uy");
            const pcl::PCLPointField& uz_field = pct::get_field(pointcloud2_with_unit_vectors, "uz");
            Eigen::Matrix<float, 3, 4> expected_unit_vectors;
            expected_unit_vectors << Eigen::Vector3f({1.f, 0.f, 0.f}), Eigen::Vector3f({0.f, 1.f, 0.f}),
                    Eigen::Vector3f({0.f, 0.f, 1.f}), Eigen::Vector3f({1.f, 1.f, 1.f}).normalized();
            for (std::size_t i = 0, j = 0; i < pointcloud2_with_unit_vectors.data.size();
                    i += pointcloud2_with_unit_vectors.point_step, ++j) {
                EXPECT_NEAR(pct::field_data<float>(&pointcloud2_with_unit_vectors.data[i + ux_field.offset],
                                    ux_field.datatype),
                        expected_unit_vectors(0, j), FLOAT_PRECISION);
                EXPECT_NEAR(pct::field_data<float>(&pointcloud2_with_unit_vectors.data[i + uy_field.offset],
                                    ux_field.datatype),
                        expected_unit_vectors(1, j), FLOAT_PRECISION);
                EXPECT_NEAR(pct::field_data<float>(&pointcloud2_with_unit_vectors.data[i + uz_field.offset],
                                    ux_field.datatype),
                        expected_unit_vectors(2, j), FLOAT_PRECISION);
            }
        }
    }
}
