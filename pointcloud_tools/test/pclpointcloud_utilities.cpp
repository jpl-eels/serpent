#include "pointcloud_tools/pclpointcloud_utilities.hpp"

#include <gtest/gtest.h>

#include "test_instances.hpp"

#define N (100)
#define FLOAT_PRECISION (1.e-7f)

TEST(check_normals, empty) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointNormal>(i);
        EXPECT_EQ(pct::check_normals(pointcloud), 0);
    }
}

TEST(check_normals, four_points_valid) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointNormal>(i);
        pointcloud.push_back(test_pn_x(i));
        pointcloud.push_back(test_pn_y(i));
        pointcloud.push_back(test_pn_z(i));
        pointcloud.push_back(test_pn_xyz(i));
        EXPECT_EQ(pct::check_normals(pointcloud), 0);
    }
}

TEST(check_normals, four_points_invalid) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointNormal>(i);
        pointcloud.push_back(test_pn_x(i));
        pointcloud.back().normal_x = 1.001;
        pointcloud.push_back(test_pn_y(i));
        pointcloud.back().normal_z -= 1.0;
        pointcloud.push_back(test_pn_z(i));
        pointcloud.back().normal_z += 1.1;
        pointcloud.back().normal_x -= 0.1;
        pointcloud.back().normal_y += 10.1;
        pointcloud.push_back(test_pn_xyz(i));
        pointcloud.back().normal_x += 1.1;
        EXPECT_EQ(pct::check_normals(pointcloud), 4);
    }
}

TEST(check_normals, valid_invalid_threshold) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointNormal>(i);
        const float threshold = 0.001f * static_cast<float>(i + 1); // Add 1 to prevent threshold of 0.
        pointcloud.push_back(test_pn_x(i)); // valid
        pointcloud.push_back(test_pn_x(i));
        pointcloud.back().normal_x += 1.1f * threshold; // invalid (note adding exactly threshold is ambiguous)
        EXPECT_EQ(pct::check_normals(pointcloud, threshold), 1);
    }
}
