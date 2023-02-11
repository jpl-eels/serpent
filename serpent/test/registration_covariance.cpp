#include "serpent/registration_covariance.hpp"

#include <gtest/gtest.h>

#define DOUBLE_PRECISION (1.0e-12)

void manual_6x6_matrix_check(const Eigen::Matrix<double, 6, 6>& m, const Eigen::Matrix<double, 6, 6>& m_expected) {
    EXPECT_NEAR(m(0, 0), m_expected(0, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(0, 1), m_expected(0, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(0, 2), m_expected(0, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(0, 3), m_expected(0, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(0, 4), m_expected(0, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(0, 5), m_expected(0, 5), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 0), m_expected(1, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 1), m_expected(1, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 2), m_expected(1, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 3), m_expected(1, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 4), m_expected(1, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(1, 5), m_expected(1, 5), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 0), m_expected(2, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 1), m_expected(2, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 2), m_expected(2, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 3), m_expected(2, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 4), m_expected(2, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(2, 5), m_expected(2, 5), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 0), m_expected(3, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 1), m_expected(3, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 2), m_expected(3, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 3), m_expected(3, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 4), m_expected(3, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(3, 5), m_expected(3, 5), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 0), m_expected(4, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 1), m_expected(4, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 2), m_expected(4, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 3), m_expected(4, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 4), m_expected(4, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(4, 5), m_expected(4, 5), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 0), m_expected(5, 0), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 1), m_expected(5, 1), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 2), m_expected(5, 2), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 3), m_expected(5, 3), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 4), m_expected(5, 4), DOUBLE_PRECISION);
    EXPECT_NEAR(m(5, 5), m_expected(5, 5), DOUBLE_PRECISION);
}

TEST(point_to_point_nonlinear, matrices) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};
    const serpent::PrecomputedTransformComponents<double> tf{r, t};

    serpent::PointToPointIcpNonlinear<pcl::PointXYZ, pcl::PointXYZ, double> cov_est;
    Eigen::Matrix<double, 6, 6> d2F_dx2 = cov_est.compute_d2F_dx2(tf, p, q);
    Eigen::Matrix<double, 6, 6> d2F_dzdx = cov_est.compute_d2F_dzdx(tf, p, q);
    Eigen::Matrix<double, 6, 6> matlab_d2F_dx2;matlab_d2F_dx2 <<
            -67.201085651061788,   5.059365828066441,  -6.360762629219814,  5.076757055895927, -3.507961143780120,  0.847498586963333,
              5.059365828066441, -13.552031654725369,  47.443877710954581,  2.496120068683950,  4.046073253100828, -2.465660350527819,
             -6.360762629219814,  47.443877710954581, -51.723464220955336, -2.616672060261007, -1.876726457606008,  1.403622544482838,
              5.076757055895927,   2.496120068683950,  -2.616672060261007,  2.000000000000000,                  0,                  0,
             -3.507961143780120,   4.046073253100828,  -1.876726457606008,                  0,  2.000000000000000,                  0,
              0.847498586963333,  -2.465660350527819,   1.403622544482838,                  0,                  0,  2.000000000000000;
    Eigen::Matrix<double, 6, 6> matlab_d2F_dzdx;
    matlab_d2F_dzdx <<
            16.334365351400059,  -3.293765751807047, -22.607690194191619, -5.076757055895927,  3.507961143780120, -0.847498586963333,
            -1.884511003104440,   2.924743219789525, -13.231204576332889, -2.496120068683950, -4.046073253100828,  2.465660350527819,
            24.742541324917347, -12.335501861357743,  11.480978748228525,  2.616672060261007,  1.876726457606008, -1.403622544482838,
            -0.631613213369898,  -1.244301744540008,   1.432751868688798, -2.000000000000000,                  0,                  0,
             1.884423877521876,  -0.233314997292292,   0.628100916942426,                  0, -2.000000000000000,                  0,
            -0.223632284167494,   1.548314335195922,   1.246078376710245,                  0,                  0, -2.000000000000000;
    manual_6x6_matrix_check(d2F_dx2, matlab_d2F_dx2);
    manual_6x6_matrix_check(d2F_dzdx, matlab_d2F_dzdx);
    EXPECT_TRUE(d2F_dx2.isApprox(matlab_d2F_dx2));
    EXPECT_TRUE(d2F_dzdx.isApprox(matlab_d2F_dzdx));
}

TEST(point_to_point_linear, matrices) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};

    serpent::PointToPointIcpLinearised<pcl::PointXYZ, pcl::PointXYZ, double> cov_est;
    Eigen::Matrix<double, 6, 6> d2F_dx2 = cov_est.compute_d2F_dx2(r, t, p, q);
    Eigen::Matrix<double, 6, 6> d2F_dzdx = cov_est.compute_d2F_dzdx(r, t, p, q);
    Eigen::Matrix<double, 6, 6> matlab_d2F_dx2;
    matlab_d2F_dx2 <<
            26,  -4,  -6,  0, -6,  4,
            -4,  20, -12,  6,  0, -2,
            -6, -12,  10, -4,  2,  0,
             0,   6,  -4,  2,  0,  0,
            -6,   0,   2,  0,  2,  0,
             4,  -2,   0,  0,  0,  2;
    Eigen::Matrix<double, 6, 6> matlab_d2F_dzdx;
    matlab_d2F_dzdx <<
            -13.800000000000001,  28.000000000000000,  -7.800000000000000,                  0,  6.000000000000000, -4.000000000000000,
            -24.200000000000000, -11.200000000000000, -16.000000000000000, -6.000000000000000,                  0,  2.000000000000000,
             14.200000000000000,  28.200000000000000,  -4.600000000000000,  4.000000000000000, -2.000000000000000,                  0,
              2.000000000000000,  -3.400000000000000,   1.800000000000000, -2.000000000000000,                  0,                  0,
              3.400000000000000,   2.000000000000000,  -1.000000000000000,                  0, -2.000000000000000,                  0,
             -1.800000000000000,   1.000000000000000,   2.000000000000000,                  0,                  0, -2.000000000000000;
    manual_6x6_matrix_check(d2F_dx2, matlab_d2F_dx2);
    manual_6x6_matrix_check(d2F_dzdx, matlab_d2F_dzdx);
    EXPECT_TRUE(d2F_dx2.isApprox(matlab_d2F_dx2));
    EXPECT_TRUE(d2F_dzdx.isApprox(matlab_d2F_dzdx));
}

TEST(point_to_plane_nonlinear, matrices) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};
    const Eigen::Vector3d n{0.455842305838552, 0.683763458757828, 0.569802882298190};
    const serpent::PrecomputedTransformComponents<double> tf{r, t};

    serpent::PointToPlaneIcpNonlinear<pcl::PointNormal, pcl::PointNormal, double> cov_est;
    Eigen::Matrix<double, 6, 6> d2F_dx2 = cov_est.compute_d2F_dx2(tf, p, q, n);
    Eigen::Matrix<double, 6, 6> d2F_dzdx = cov_est.compute_d2F_dzdx(tf, p, q, n);
    Eigen::Matrix<double, 6, 6> matlab_d2F_dx2;
    matlab_d2F_dx2 <<
            -39.100196173508898,   9.861356916973952, 17.770943495444232,  0.181649573803618,  0.272474360705427,  0.227061967254523,
              9.861356916973952, -36.687260600053314, 12.337082744281018,  1.139356781335152,  1.709035172002728,  1.424195976668940,
             17.770943495444232,  12.337082744281018, -9.904373831824673, -0.764100481260566, -1.146150721890848, -0.955125601575707,
              0.181649573803618,   1.139356781335152, -0.764100481260566,  0.415584415584416,  0.623376623376623,  0.519480519480519,
              0.272474360705427,   1.709035172002728, -1.146150721890848,  0.623376623376623,  0.935064935064935,  0.779220779220779,
              0.227061967254523,   1.424195976668940, -0.955125601575707,  0.519480519480519,  0.779220779220779,  0.649350649350649;
    Eigen::Matrix<double, 6, 6> matlab_d2F_dzdx;
    matlab_d2F_dzdx <<
            10.200317439663957,   4.894710271757694, -4.948460499948905, -0.181649573803618, -0.272474360705427, -0.227061967254523,
            -8.714359476803656,  11.427524352097938,  6.042049727029035, -1.139356781335152, -1.709035172002728, -1.424195976668940,
            -6.130094291272190, -10.838560868316518,  2.055906604465768,  0.764100481260566,  1.146150721890848,  0.955125601575707,
             0.398022285237101,   0.070881803328095,  0.817143109608362, -0.415584415584416, -0.623376623376623, -0.519480519480519,
             0.597033427855651,   0.106322704992142,  1.225714664412544, -0.623376623376623, -0.935064935064935, -0.779220779220779,
             0.497527856546376,   0.088602254160118,  1.021428887010453, -0.519480519480519, -0.779220779220779, -0.649350649350649;
    manual_6x6_matrix_check(d2F_dx2, matlab_d2F_dx2);
    manual_6x6_matrix_check(d2F_dzdx, matlab_d2F_dzdx);
    EXPECT_TRUE(d2F_dx2.isApprox(matlab_d2F_dx2));
    EXPECT_TRUE(d2F_dzdx.isApprox(matlab_d2F_dzdx));
}

TEST(point_to_plane_linear, matrices) {
    const Eigen::Vector3d r{0.5, 0.9, 1.7};
    const Eigen::Vector3d t{-4.5, 14.0, 8.9};
    const Eigen::Vector3d p{1.0, 2.0, 3.0};
    const Eigen::Vector3d q{5.5, 8.8, -4.0};
    const Eigen::Vector3d n{0.455842305838552, 0.683763458757828, 0.569802882298190};

    serpent::PointToPlaneIcpLinearised<pcl::PointNormal, pcl::PointNormal, double> cov_est;
    Eigen::Matrix<double, 6, 6> d2F_dx2 = cov_est.compute_d2F_dx2(r, t, p, q, n);
    Eigen::Matrix<double, 6, 6> d2F_dzdx = cov_est.compute_d2F_dzdx(r, t, p, q, n);
    Eigen::Matrix<double, 6, 6> matlab_d2F_dx2;
    matlab_d2F_dx2 <<
             1.662337662337662, -1.454545454545455,  0.415584415584416, -0.831168831168831, -1.246753246753247, -1.038961038961039,
            -1.454545454545455,  1.272727272727273, -0.363636363636364,  0.727272727272727,  1.090909090909091,  0.909090909090909,
             0.415584415584416, -0.363636363636364,  0.103896103896104, -0.207792207792208, -0.311688311688312, -0.259740259740260,
            -0.831168831168831,  0.727272727272727, -0.207792207792208,  0.415584415584416,  0.623376623376623,  0.519480519480519,
            -1.246753246753247,  1.090909090909091, -0.311688311688312,  0.623376623376623,  0.935064935064935,  0.779220779220779,
            -1.038961038961039,  0.909090909090909, -0.259740259740260,  0.519480519480519,  0.779220779220779,  0.649350649350649;
    Eigen::Matrix<double, 6, 6> matlab_d2F_dzdx;
    matlab_d2F_dzdx <<
            -2.015584415584415, 10.763636363636364, -14.503896103896103,  0.831168831168831,  1.246753246753247,  1.038961038961039,
            -9.353246753246752,  0.309090909090909,   9.911688311688312, -0.727272727272727, -1.090909090909091, -0.909090909090909,
            12.836363636363636, -8.981818181818182,  -0.290909090909091,  0.207792207792208,  0.311688311688312,  0.259740259740260,
             1.007792207792208,  0.176623376623377,   0.581818181818182, -0.415584415584416, -0.623376623376623, -0.519480519480519,
             1.511688311688312,  0.264935064935065,   0.872727272727273, -0.623376623376623, -0.935064935064935, -0.779220779220779,
             1.259740259740260,  0.220779220779221,   0.727272727272727, -0.519480519480519, -0.779220779220779, -0.649350649350649;
    manual_6x6_matrix_check(d2F_dx2, matlab_d2F_dx2);
    manual_6x6_matrix_check(d2F_dzdx, matlab_d2F_dzdx);
    EXPECT_TRUE(d2F_dx2.isApprox(matlab_d2F_dx2));
    EXPECT_TRUE(d2F_dzdx.isApprox(matlab_d2F_dzdx));
}
