#include "pointcloud_tools/pclpointcloud2_utilities.hpp"

#include <gtest/gtest.h>

#include "eigen_ext/covariance.hpp"
#include "eigen_ext/geometry.hpp"
#include "test_instances.hpp"

#define N (10)
#define FLOAT_PRECISION (1.e-7f)
#define RELAXED_FLOAT_PRECISION (2.e-5f)

TEST(add_fields, float32_fields) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 = pct::add_fields(pcl::PCLPointCloud2(), {"a", "b", "c"},
                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1);
        EXPECT_TRUE(pct::has_field(pointcloud2, "a"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "b"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "c"));
    }
}

TEST(add_fields, int32_fields) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 =
                pct::add_fields(pcl::PCLPointCloud2(), {"a", "b", "c"}, pcl::PCLPointField::PointFieldTypes::INT32, 1);
        EXPECT_TRUE(pct::has_field(pointcloud2, "a"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "b"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "c"));
    }
}

TEST(add_fields, xyz_plus_float32_fields) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 = test_empty_pointcloud2<pcl::PointXYZ>(i);
        pointcloud2 = pct::add_fields(pointcloud2, {"a", "b", "c"}, pcl::PCLPointField::PointFieldTypes::FLOAT32, 1);
        EXPECT_TRUE(pct::has_field(pointcloud2, "a"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "b"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "c"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "x"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "y"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "z"));
    }
}

TEST(add_fields, xyz_plus_int32_fields) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 = test_empty_pointcloud2<pcl::PointXYZ>(i);
        pointcloud2 = pct::add_fields(pointcloud2, {"a", "b", "c"}, pcl::PCLPointField::PointFieldTypes::INT32, 1);
        EXPECT_TRUE(pct::has_field(pointcloud2, "a"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "b"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "c"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "x"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "y"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "z"));
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
            const std::size_t num_points = pct::size_points(pointcloud2_with_unit_vectors);
            for (std::size_t i = 0; i < num_points; ++i) {
                EXPECT_NEAR(pct::field_data<float>(pointcloud2_with_unit_vectors, ux_field, i),
                        expected_unit_vectors(0, i), FLOAT_PRECISION);
                EXPECT_NEAR(pct::field_data<float>(pointcloud2_with_unit_vectors, uy_field, i),
                        expected_unit_vectors(1, i), FLOAT_PRECISION);
                EXPECT_NEAR(pct::field_data<float>(pointcloud2_with_unit_vectors, uz_field, i),
                        expected_unit_vectors(2, i), FLOAT_PRECISION);
            }
        }
    }
}

TEST(cast, pointxyz_float32_to_int32) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(pcl::PointXYZ{-1.f, 2.f, 3.f});
        pointcloud.push_back(pcl::PointXYZ{-0.7f, -2.2f, 4.4f});
        pointcloud.push_back(pcl::PointXYZ{1.5f, 1032.f, 6.9f});
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "x");
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "y");
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "z");
        auto get_x_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto get_y_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto get_z_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "z"));
        for (std::size_t i = 0; i < pointcloud.size(); ++i) {
            EXPECT_EQ(get_x_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].x));
            EXPECT_EQ(get_y_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].y));
            EXPECT_EQ(get_z_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].z));
        }
    }
}

TEST(cast, pointxyz_int32_to_float32) {
    for (unsigned int i = 0; i < N; ++i) {
        pcl::PCLPointCloud2 pointcloud2 =
                pct::add_fields(pcl::PCLPointCloud2(), {"x", "y", "z"}, pcl::PCLPointField::PointFieldTypes::INT32);
        auto set_x_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto set_y_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto set_z_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "z"));
        pct::resize(pointcloud2, 3);
        set_x_field_data(pointcloud2, 0, 1);
        set_y_field_data(pointcloud2, 0, 2);
        set_z_field_data(pointcloud2, 0, 3);
        set_x_field_data(pointcloud2, 1, -3);
        set_y_field_data(pointcloud2, 1, -2);
        set_z_field_data(pointcloud2, 1, -1);
        set_x_field_data(pointcloud2, 2, 10);
        set_y_field_data(pointcloud2, 2, -4);
        set_z_field_data(pointcloud2, 2, 0);
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "x");
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "y");
        pct::cast_field<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "z");
        auto get_x_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto get_y_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto get_z_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "z"));
        EXPECT_NEAR(get_x_field_data(pointcloud2, 0), static_cast<float>(1), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 0), static_cast<float>(2), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 0), static_cast<float>(3), FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2, 1), static_cast<float>(-3), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 1), static_cast<float>(-2), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 1), static_cast<float>(-1), FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2, 2), static_cast<float>(10), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 2), static_cast<float>(-4), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 2), static_cast<float>(0), FLOAT_PRECISION);
    }
}

TEST(cast_field_with_scale, pointxyz_float32_to_int32) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(pcl::PointXYZ{-1.f, 2.f, 3.f});
        pointcloud.push_back(pcl::PointXYZ{-0.7f, -2.2f, 4.4f});
        pointcloud.push_back(pcl::PointXYZ{1.5f, 1032.f, 6.9f});
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        const double scale = static_cast<double>(i);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "x", scale);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "y", scale);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::INT32>(pointcloud2, "z", scale);
        auto get_x_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto get_y_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto get_z_field_data = pct::create_get_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "z"));
        for (std::size_t i = 0; i < pointcloud.size(); ++i) {
            EXPECT_EQ(get_x_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].x * scale));
            EXPECT_EQ(get_y_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].y * scale));
            EXPECT_EQ(get_z_field_data(pointcloud2, i), static_cast<int>(pointcloud[i].z * scale));
        }
    }
}

TEST(cast_field_with_scale, pointxyz_int32_to_float32) {
    for (unsigned int i = 0; i < N; ++i) {
        pcl::PCLPointCloud2 pointcloud2 =
                pct::add_fields(pcl::PCLPointCloud2(), {"x", "y", "z"}, pcl::PCLPointField::PointFieldTypes::INT32);
        auto set_x_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto set_y_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto set_z_field_data = pct::create_set_field_data_function<int,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::INT32>::type>(
                pct::get_field(pointcloud2, "z"));
        pct::resize(pointcloud2, 3);
        set_x_field_data(pointcloud2, 0, 1);
        set_y_field_data(pointcloud2, 0, 2);
        set_z_field_data(pointcloud2, 0, 3);
        set_x_field_data(pointcloud2, 1, -3);
        set_y_field_data(pointcloud2, 1, -2);
        set_z_field_data(pointcloud2, 1, -1);
        set_x_field_data(pointcloud2, 2, 10);
        set_y_field_data(pointcloud2, 2, -4);
        set_z_field_data(pointcloud2, 2, 0);
        const double scale = static_cast<double>(i);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "x", scale);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "y", scale);
        pct::cast_field_with_scale<pcl::PCLPointField::PointFieldTypes::FLOAT32>(pointcloud2, "z", scale);
        auto get_x_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "x"));
        auto get_y_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "y"));
        auto get_z_field_data = pct::create_get_field_data_function<float,
                pcl::traits::asType<pcl::PCLPointField::PointFieldTypes::FLOAT32>::type>(
                pct::get_field(pointcloud2, "z"));
        EXPECT_NEAR(get_x_field_data(pointcloud2, 0), static_cast<float>(1 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 0), static_cast<float>(2 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 0), static_cast<float>(3 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2, 1), static_cast<float>(-3 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 1), static_cast<float>(-2 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 1), static_cast<float>(-1 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2, 2), static_cast<float>(10 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 2), static_cast<float>(-4 * scale), FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 2), static_cast<float>(0 * scale), FLOAT_PRECISION);
    }
}

TEST(change_field_name, point_xyz) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 = test_empty_pointcloud2<pcl::PointXYZ>(i);
        EXPECT_TRUE(pct::has_field(pointcloud2, "x"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "y"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "z"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "new_x"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "new_y"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "new_z"));
        pct::change_field_name(pointcloud2, "x", "new_x");
        pct::change_field_name(pointcloud2, "y", "new_y");
        pct::change_field_name(pointcloud2, "z", "new_z");
        EXPECT_TRUE(pct::has_field(pointcloud2, "new_x"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "new_y"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "new_z"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "x"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "y"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "z"));
    }
}

TEST(deskew, pointxyz_translation_to_start) {
    for (unsigned int i = 0; i < N; ++i) {
        // Create pointcloud with 3 points
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_xyz(static_cast<float>(i + 0)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 2)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        pointcloud2 = pct::add_field(pointcloud2, "t", pcl::PCLPointField::PointFieldTypes::FLOAT32);
        auto x_field = pct::get_field(pointcloud2, "x");
        auto y_field = pct::get_field(pointcloud2, "y");
        auto z_field = pct::get_field(pointcloud2, "z");
        auto t_field = pct::get_field(pointcloud2, "t");
        auto get_x_field_data = pct::create_get_field_data_function<float>(x_field);
        auto get_y_field_data = pct::create_get_field_data_function<float>(y_field);
        auto get_z_field_data = pct::create_get_field_data_function<float>(z_field);
        auto get_t_field_data = pct::create_get_field_data_function<float>(t_field);
        pct::set_field_data(pointcloud2, t_field, 0, 0.0f);
        pct::set_field_data(pointcloud2, t_field, 1, 0.05f);
        pct::set_field_data(pointcloud2, t_field, 2, 0.1f);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 1), 0.05f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 2), 0.1f, FLOAT_PRECISION);

        Eigen::Vector3d translation{1.0, 2.0, 3.0};
        Eigen::Isometry3d tf = eigen_ext::to_transform(translation, Eigen::Quaterniond::Identity());
        const auto new_time = pointcloud2.header.stamp;
        pcl::PCLPointCloud2 pointcloud2_deskewed;
        pct::deskew(tf, 0.1, new_time, pointcloud2, pointcloud2_deskewed);
        EXPECT_EQ(pointcloud2_deskewed.header.stamp, new_time);
        EXPECT_EQ(pointcloud2_deskewed.header.frame_id, pointcloud2.header.frame_id);
        EXPECT_EQ(pct::size_points(pointcloud2_deskewed), pct::size_points(pointcloud2));
        EXPECT_EQ(pct::size_bytes(pointcloud2_deskewed), pct::size_bytes(pointcloud2));
        EXPECT_EQ(pointcloud2_deskewed.height, pointcloud2.height);
        EXPECT_EQ(pointcloud2_deskewed.width, pointcloud2.width);
        EXPECT_EQ(pointcloud2_deskewed.fields, pointcloud2.fields);
        EXPECT_EQ(pointcloud2_deskewed.is_bigendian, pointcloud2.is_bigendian);
        EXPECT_EQ(pointcloud2_deskewed.point_step, pointcloud2.point_step);
        EXPECT_EQ(pointcloud2_deskewed.row_step, pointcloud2.row_step);
        EXPECT_EQ(pointcloud2_deskewed.is_dense, pointcloud2.is_dense);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 0), pointcloud[0].x, FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 0), pointcloud[0].y, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 0), pointcloud[0].z, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 1), pointcloud[1].z + 0.5 * translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 1), pointcloud[1].y + 0.5 * translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 1), pointcloud[1].x + 0.5 * translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 1), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 2), pointcloud[2].x + translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 2), pointcloud[2].y + translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 2), pointcloud[2].z + translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 2), 0.0f, FLOAT_PRECISION);
    }
}

TEST(deskew, pointxyz_translation_to_mid) {
    for (unsigned int i = 0; i < N; ++i) {
        // Create pointcloud with 3 points
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_xyz(static_cast<float>(i + 0)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 2)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        pointcloud2 = pct::add_field(pointcloud2, "t", pcl::PCLPointField::PointFieldTypes::FLOAT32);
        auto x_field = pct::get_field(pointcloud2, "x");
        auto y_field = pct::get_field(pointcloud2, "y");
        auto z_field = pct::get_field(pointcloud2, "z");
        auto t_field = pct::get_field(pointcloud2, "t");
        auto get_x_field_data = pct::create_get_field_data_function<float>(x_field);
        auto get_y_field_data = pct::create_get_field_data_function<float>(y_field);
        auto get_z_field_data = pct::create_get_field_data_function<float>(z_field);
        auto get_t_field_data = pct::create_get_field_data_function<float>(t_field);
        pct::set_field_data(pointcloud2, t_field, 0, 0.0f);
        pct::set_field_data(pointcloud2, t_field, 1, 0.05f);
        pct::set_field_data(pointcloud2, t_field, 2, 0.1f);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 1), 0.05f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 2), 0.1f, FLOAT_PRECISION);

        Eigen::Vector3d translation{1.0, 2.0, 3.0};
        Eigen::Isometry3d tf = eigen_ext::to_transform(translation, Eigen::Quaterniond::Identity());
        const auto new_time = pointcloud2.header.stamp + 50000;  // 0.05s = 50000 us
        pcl::PCLPointCloud2 pointcloud2_deskewed;
        pct::deskew(tf, 0.1, new_time, pointcloud2, pointcloud2_deskewed);
        EXPECT_EQ(pointcloud2_deskewed.header.stamp, new_time);
        EXPECT_EQ(pointcloud2_deskewed.header.frame_id, pointcloud2.header.frame_id);
        EXPECT_EQ(pct::size_points(pointcloud2_deskewed), pct::size_points(pointcloud2));
        EXPECT_EQ(pct::size_bytes(pointcloud2_deskewed), pct::size_bytes(pointcloud2));
        EXPECT_EQ(pointcloud2_deskewed.height, pointcloud2.height);
        EXPECT_EQ(pointcloud2_deskewed.width, pointcloud2.width);
        EXPECT_EQ(pointcloud2_deskewed.fields, pointcloud2.fields);
        EXPECT_EQ(pointcloud2_deskewed.is_bigendian, pointcloud2.is_bigendian);
        EXPECT_EQ(pointcloud2_deskewed.point_step, pointcloud2.point_step);
        EXPECT_EQ(pointcloud2_deskewed.row_step, pointcloud2.row_step);
        EXPECT_EQ(pointcloud2_deskewed.is_dense, pointcloud2.is_dense);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 0), pointcloud[0].x - 0.5 * translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 0), pointcloud[0].y - 0.5 * translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 0), pointcloud[0].z - 0.5 * translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 1), pointcloud[1].x, FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 1), pointcloud[1].y, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 1), pointcloud[1].z, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 1), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 2), pointcloud[2].x + 0.5 * translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 2), pointcloud[2].y + 0.5 * translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 2), pointcloud[2].z + 0.5 * translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 2), 0.0f, FLOAT_PRECISION);
    }
}

TEST(deskew, pointxyz_translation_to_end) {
    for (unsigned int i = 0; i < N; ++i) {
        // Create pointcloud with 3 points
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_xyz(static_cast<float>(i + 0)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 2)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        pointcloud2 = pct::add_field(pointcloud2, "t", pcl::PCLPointField::PointFieldTypes::FLOAT32);
        auto x_field = pct::get_field(pointcloud2, "x");
        auto y_field = pct::get_field(pointcloud2, "y");
        auto z_field = pct::get_field(pointcloud2, "z");
        auto t_field = pct::get_field(pointcloud2, "t");
        auto get_x_field_data = pct::create_get_field_data_function<float>(x_field);
        auto get_y_field_data = pct::create_get_field_data_function<float>(y_field);
        auto get_z_field_data = pct::create_get_field_data_function<float>(z_field);
        auto get_t_field_data = pct::create_get_field_data_function<float>(t_field);
        pct::set_field_data(pointcloud2, t_field, 0, 0.0f);
        pct::set_field_data(pointcloud2, t_field, 1, 0.05f);
        pct::set_field_data(pointcloud2, t_field, 2, 0.1f);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 1), 0.05f, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2, 2), 0.1f, FLOAT_PRECISION);

        Eigen::Vector3d translation{1.0, 2.0, 3.0};
        Eigen::Isometry3d tf = eigen_ext::to_transform(translation, Eigen::Quaterniond::Identity());
        const auto new_time = pointcloud2.header.stamp + 100000;  // 0.1s = 100000 us
        pcl::PCLPointCloud2 pointcloud2_deskewed;
        pct::deskew(tf, 0.1, new_time, pointcloud2, pointcloud2_deskewed);
        EXPECT_EQ(pointcloud2_deskewed.header.stamp, new_time);
        EXPECT_EQ(pointcloud2_deskewed.header.frame_id, pointcloud2.header.frame_id);
        EXPECT_EQ(pct::size_points(pointcloud2_deskewed), pct::size_points(pointcloud2));
        EXPECT_EQ(pct::size_bytes(pointcloud2_deskewed), pct::size_bytes(pointcloud2));
        EXPECT_EQ(pointcloud2_deskewed.height, pointcloud2.height);
        EXPECT_EQ(pointcloud2_deskewed.width, pointcloud2.width);
        EXPECT_EQ(pointcloud2_deskewed.fields, pointcloud2.fields);
        EXPECT_EQ(pointcloud2_deskewed.is_bigendian, pointcloud2.is_bigendian);
        EXPECT_EQ(pointcloud2_deskewed.point_step, pointcloud2.point_step);
        EXPECT_EQ(pointcloud2_deskewed.row_step, pointcloud2.row_step);
        EXPECT_EQ(pointcloud2_deskewed.is_dense, pointcloud2.is_dense);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 0), pointcloud[0].x - translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 0), pointcloud[0].y - translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 0), pointcloud[0].z - translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 0), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 1), pointcloud[1].x - 0.5 * translation[0], FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 1), pointcloud[1].y - 0.5 * translation[1], FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 1), pointcloud[1].z - 0.5 * translation[2], FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 1), 0.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2_deskewed, 2), pointcloud[2].x, FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2_deskewed, 2), pointcloud[2].y, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2_deskewed, 2), pointcloud[2].z, FLOAT_PRECISION);
        EXPECT_NEAR(get_t_field_data(pointcloud2_deskewed, 2), 0.0f, FLOAT_PRECISION);
    }
}

TEST(empty, pointxyz) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_TRUE(pct::empty(pointcloud2));
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_FALSE(pct::empty(pointcloud2));
    }
}

TEST(filter_max, pointxyz_double) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(pcl::PointXYZ{-1.f, 2.f, 3.f});
        pointcloud.push_back(pcl::PointXYZ{-0.7f, -2.2f, 4.4f});
        pointcloud.push_back(pcl::PointXYZ{1.5f, 1032.f, 6.9f});
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        pcl::PCLPointCloud2 pointcloud2_x, pointcloud2_y, pointcloud2_z;
        pct::filter_max<double>(pointcloud2, pointcloud2_x, "x", -0.6f);
        pct::filter_max<double>(pointcloud2, pointcloud2_y, "y", 1.9f);
        pct::filter_max<double>(pointcloud2, pointcloud2_z, "z", 2.9f);
        for (std::size_t i = 0; i < pointcloud.size(); ++i) {
            EXPECT_EQ(pct::size_points(pointcloud2_x), 2);
            EXPECT_EQ(pct::size_points(pointcloud2_y), 1);
            EXPECT_EQ(pct::size_points(pointcloud2_z), 0);
        }
    }
}

TEST(get_set_field_data_functions, pointxyz) {
    for (unsigned int i = 0; i < N; ++i) {
        // Create pointcloud with 3 points
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        auto x_field = pct::get_field(pointcloud2, "x");
        auto y_field = pct::get_field(pointcloud2, "y");
        auto z_field = pct::get_field(pointcloud2, "z");
        auto get_x_field_data = pct::create_get_field_data_function<float>(x_field);
        auto get_y_field_data = pct::create_get_field_data_function<float>(y_field);
        auto get_z_field_data = pct::create_get_field_data_function<float>(z_field);
        auto set_x_field_data = pct::create_set_field_data_function<float>(x_field);
        auto set_y_field_data = pct::create_set_field_data_function<float>(y_field);
        auto set_z_field_data = pct::create_set_field_data_function<float>(z_field);

        // Set
        set_x_field_data(pointcloud2, 0, 4.f);
        set_y_field_data(pointcloud2, 0, 5.f);
        set_z_field_data(pointcloud2, 0, 6.f);
        set_x_field_data(pointcloud2, 1, 7.f);
        set_y_field_data(pointcloud2, 1, 8.f);
        set_z_field_data(pointcloud2, 1, 9.f);

        // Get
        EXPECT_NEAR(get_x_field_data(pointcloud2, 0), 4.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 0), 5.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 0), 6.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_x_field_data(pointcloud2, 1), 7.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_y_field_data(pointcloud2, 1), 8.0f, FLOAT_PRECISION);
        EXPECT_NEAR(get_z_field_data(pointcloud2, 1), 9.0f, FLOAT_PRECISION);

        // Set
        pct::set_field_data(pointcloud2, x_field, 0, 10.f);
        pct::set_field_data(pointcloud2, y_field, 0, 11.f);
        pct::set_field_data(pointcloud2, z_field, 0, 12.f);
        pct::set_field_data(pointcloud2, x_field, 1, 13.f);
        pct::set_field_data(pointcloud2, y_field, 1, 14.f);
        pct::set_field_data(pointcloud2, z_field, 1, 15.f);

        // Get
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, x_field, 0), 10.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, y_field, 0), 11.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, z_field, 0), 12.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, x_field, 1), 13.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, y_field, 1), 14.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::field_data<float>(pointcloud2, z_field, 1), 15.f, FLOAT_PRECISION);

        // Convert
        pcl::PointCloud<pcl::PointXYZ> pointcloud_post;
        pcl::fromPCLPointCloud2(pointcloud2, pointcloud_post);
        EXPECT_NEAR(pointcloud_post[0].x, 10.f, FLOAT_PRECISION);
        EXPECT_NEAR(pointcloud_post[0].y, 11.f, FLOAT_PRECISION);
        EXPECT_NEAR(pointcloud_post[0].z, 12.f, FLOAT_PRECISION);
        EXPECT_NEAR(pointcloud_post[1].x, 13.f, FLOAT_PRECISION);
        EXPECT_NEAR(pointcloud_post[1].y, 14.f, FLOAT_PRECISION);
        EXPECT_NEAR(pointcloud_post[1].z, 15.f, FLOAT_PRECISION);
    }
}

TEST(has_field, point_xyz) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud2 = test_empty_pointcloud2<pcl::PointXYZ>(i);
        EXPECT_TRUE(pct::has_field(pointcloud2, "x"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "y"));
        EXPECT_TRUE(pct::has_field(pointcloud2, "z"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "x "));
        EXPECT_FALSE(pct::has_field(pointcloud2, "x_"));
        EXPECT_FALSE(pct::has_field(pointcloud2, " x"));
        EXPECT_FALSE(pct::has_field(pointcloud2, " x "));
        EXPECT_FALSE(pct::has_field(pointcloud2, "_x"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "xx"));
        EXPECT_FALSE(pct::has_field(pointcloud2, "t"));
        EXPECT_FALSE(pct::has_field(pointcloud2, ""));
    }
}

TEST(max, pointxyz_floating_point) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(pcl::PointXYZ{-1.f, 2.f, 3.f});
        pointcloud.push_back(pcl::PointXYZ{-0.7f, -2.f, 4.f});
        pointcloud.push_back(pcl::PointXYZ{-0.4f, -12.f, 5.f});
        pointcloud.push_back(pcl::PointXYZ{-4.3f, 2.f, -5.f});
        pointcloud.push_back(pcl::PointXYZ{-2.2f, -12.f, 5.f});
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_NEAR(pct::max<float>(pointcloud2, "x"), -0.4f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::max<double>(pointcloud2, "x"), -0.4, FLOAT_PRECISION);
        EXPECT_NEAR(pct::max<float>(pointcloud2, "y"), 2.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::max<double>(pointcloud2, "y"), 2.0, FLOAT_PRECISION);
        EXPECT_NEAR(pct::max<float>(pointcloud2, "z"), 5.f, FLOAT_PRECISION);
        EXPECT_NEAR(pct::max<double>(pointcloud2, "z"), 5.0, FLOAT_PRECISION);
    }
}

TEST(scale_field, pointxyz) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pointcloud.push_back(pcl::PointXYZ{-1.f, 2.f, 3.f});
        pointcloud.push_back(pcl::PointXYZ{-0.7f, -2.2f, 4.4f});
        pointcloud.push_back(pcl::PointXYZ{1.5f, 43.2f, 6.9f});
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        const double scale = static_cast<double>(i);
        pct::scale_field(pointcloud2, "x", scale);
        pct::scale_field(pointcloud2, "y", scale);
        pct::scale_field(pointcloud2, "z", scale);
        auto get_x_field_data = pct::create_get_field_data_function<float>(pct::get_field(pointcloud2, "x"));
        auto get_y_field_data = pct::create_get_field_data_function<float>(pct::get_field(pointcloud2, "y"));
        auto get_z_field_data = pct::create_get_field_data_function<float>(pct::get_field(pointcloud2, "z"));
        for (std::size_t i = 0; i < pointcloud.size(); ++i) {
            EXPECT_NEAR(get_x_field_data(pointcloud2, i), pointcloud[i].x * scale, RELAXED_FLOAT_PRECISION);
            EXPECT_NEAR(get_y_field_data(pointcloud2, i), pointcloud[i].y * scale, RELAXED_FLOAT_PRECISION);
            EXPECT_NEAR(get_z_field_data(pointcloud2, i), pointcloud[i].z * scale, RELAXED_FLOAT_PRECISION);
        }
    }
}

TEST(size_points, pointxyz) {
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pct::size_points(pointcloud2), 0);
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pct::size_points(pointcloud2), 1);
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pct::size_points(pointcloud2), 2);
    }
}

TEST(size_bytes, pointxyz) {
    const std::size_t point_size = 4 * sizeof(float);
    for (unsigned int i = 0; i < N; ++i) {
        auto pointcloud = test_empty_pointcloud<pcl::PointXYZ>(i);
        pcl::PCLPointCloud2 pointcloud2;
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pointcloud2.point_step, point_size);
        EXPECT_EQ(pct::size_bytes(pointcloud2), 0);
        pointcloud.push_back(test_xyz(static_cast<float>(i)));
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pct::size_bytes(pointcloud2), 1 * point_size);
        pointcloud.push_back(test_xyz(static_cast<float>(i + 1)));
        pcl::toPCLPointCloud2(pointcloud, pointcloud2);
        EXPECT_EQ(pct::size_bytes(pointcloud2), 2 * point_size);
    }
}

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
