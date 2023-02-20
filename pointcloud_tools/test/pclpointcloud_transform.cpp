#include <gtest/gtest.h>

#include "eigen_ext/geometry.hpp"
#include "pointcloud_tools/pclpointcloud_transform.hpp"
#include "pointcloud_tools/point_types.hpp"
#include "test_instances.hpp"

#define N (100)
#define FLOAT_PRECISION (1.e-7f)
#define RELAXED_FLOAT_PRECISION (2.e-5f)

TEST(transform_pointcloud_with_normals_and_unit_vectors, four_point_cloud) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<PointNormalUnit>(i);
        pointcloud.push_back(test_pnu_x(i));
        pointcloud.push_back(test_pnu_y(i));
        pointcloud.push_back(test_pnu_z(i));
        pointcloud.push_back(test_pnu_xyz(i));
        pcl::PointCloud<PointNormalUnit> transformed_pointcloud;
        Eigen::Vector3f t{2.f, 3.f, 4.f};
        Eigen::Quaternionf q = Eigen::Quaternionf{1.f, 2.f, 3.f, 4.f}.normalized();
        Eigen::Transform<float, 3, Eigen::Isometry> tf = eigen_ext::to_transform(t, q);
        pct::transform_pointcloud_with_normals_and_unit_vectors(pointcloud, transformed_pointcloud, tf.matrix(), true);
        EXPECT_EQ(transformed_pointcloud.size(), pointcloud.size());
        for (std::size_t i = 0; i < pointcloud.size(); ++i) {
            const Eigen::Vector3f original_p = Eigen::Vector3f{pointcloud[i].x, pointcloud[i].y, pointcloud[i].z};
            const Eigen::Vector3f expected_p = tf * original_p;
            EXPECT_NEAR(transformed_pointcloud[i].x, expected_p[0], RELAXED_FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].y, expected_p[1], RELAXED_FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].z, expected_p[2], RELAXED_FLOAT_PRECISION);
            const Eigen::Vector3f original_n =
                    Eigen::Vector3f{pointcloud[i].normal_x, pointcloud[i].normal_y, pointcloud[i].normal_z};
            const Eigen::Vector3f expected_n = q * original_n;
            EXPECT_NEAR(transformed_pointcloud[i].normal_x, expected_n[0], FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].normal_y, expected_n[1], FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].normal_z, expected_n[2], FLOAT_PRECISION);
            const Eigen::Vector3f original_u =
                    Eigen::Vector3f{pointcloud[i].normal_x, pointcloud[i].normal_y, pointcloud[i].normal_z};
            const Eigen::Vector3f expected_u = q * original_u;
            EXPECT_NEAR(transformed_pointcloud[i].normal_x, expected_u[0], FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].normal_y, expected_u[1], FLOAT_PRECISION);
            EXPECT_NEAR(transformed_pointcloud[i].normal_z, expected_u[2], FLOAT_PRECISION);
        }
    }
}
