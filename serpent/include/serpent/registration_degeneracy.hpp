#ifndef SERPENT_REGISTRATION_DEGENERACY_HPP
#define SERPENT_REGISTRATION_DEGENERACY_HPP

#include <Eigen/Geometry>
#include <pcl/common/distances.h>
#include <pcl/registration/registration.h>

namespace serpent {

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> point_to_point_jacobian(const Eigen::Matrix<Scalar, 3, 3>& R,
        const Eigen::Matrix<Scalar, 3, 1>& t, const Eigen::Matrix<Scalar, 3, 1>& r, const Scalar sina,
        const Scalar cosa, const Eigen::Matrix<Scalar, 3, 1>& p, const Eigen::Matrix<Scalar, 3, 1>& q);

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> point_to_plane_jacobian(const Eigen::Matrix<Scalar, 3, 1>& t,
        const Eigen::Matrix<Scalar, 3, 1>& r, const Eigen::Matrix<Scalar, 3, 1>& p,
        const Eigen::Matrix<Scalar, 3, 1>& q, const Eigen::Matrix<Scalar, 3, 1>& n);

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 1> point_to_point_jacobian(typename pcl::Registration<PointIn, PointOut>& registration, int& count);

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 1> point_to_plane_jacobian(typename pcl::Registration<PointIn, PointOut>& registration, int& count);

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 6> point_to_point_jacobian_matrix(
        typename pcl::Registration<PointIn, PointOut>& registration, int& count);

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 6> point_to_plane_jacobian_matrix(
        typename pcl::Registration<PointIn, PointOut>& registration, int& count);

/* Implementation */

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> point_to_point_jacobian(const Eigen::Matrix<Scalar, 3, 3>& R,
        const Eigen::Matrix<Scalar, 3, 1>& t, const Eigen::Matrix<Scalar, 3, 1>& r, const Scalar sina,
        const Scalar cosa, const Eigen::Matrix<Scalar, 3, 1>& p, const Eigen::Matrix<Scalar, 3, 1>& q) {
    // Compute G
    const Eigen::Matrix<Scalar, 3, 1> G = R * p + t - q;
    
    // Compute dG_dx
    Eigen::Matrix<Scalar, 3, 6> dG_dx = Eigen::Matrix<Scalar, 3, 6>::Zero();
    const Scalar cosa_m1 = cosa - 1;
    const Scalar a2 = r.squaredNorm();
    const Scalar a = r.norm();
    const Scalar a2_3on2 = a2 * a;
    const Scalar a2_2 = a2 * a2;
    const Scalar rx2 = r[0] * r[0];
    const Scalar rx3 = rx2 * r[0];
    const Scalar ry2 = r[1] * r[1];
    const Scalar ry3 = ry2 * r[1];
    const Scalar rz2 = r[2] * r[2];
    const Scalar rz3 = rz2 * r[2];
    const Scalar rxryrz = r[0] * r[1] * r[2];
    /*
           /   3                     3                                 \
           | rx  sin(sqrt(#2))   2 rx  #1   rx sin(sqrt(#2))   2 rx #1 |
        px | ----------------- + -------- - ---------------- - ------- |
           |         3/2              2         sqrt(#2)          #2   |
           \       #2               #2                                 /
        
                /                                                       2                        2       \
                | rx rz sin(sqrt(#2))   rx rz cos(sqrt(#2))   ry #1   rx  ry sin(sqrt(#2))   2 rx  ry #1 |
           + py | ------------------- - ------------------- - ----- + -------------------- + ----------- |
                |          3/2                   #2             #2              3/2                2     |
                \        #2                                                   #2                 #2      /
        
                /                                                         2                        2       \
                |   rx ry cos(sqrt(#2))   rz #1   rx ry cos(sqrt(#2))   rx  rz sin(sqrt(#2))   2 rx  rz #1 |
           + pz | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \           #2                                                  #2                 #2      /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(0, 0) = p[0] * ((rx3 * sina) / a2_3on2 + (2.0 * rx3 * cosa_m1) / a2_2 - (r[0] * sina) / a - (2.0 * r[0] * cosa_m1) / a2)
        + p[1] * ((r[0] * r[2] * sina) / a2_3on2 - (r[0] * r[2] * cosa) / a2 - (r[1] * cosa_m1) / a2 + (rx2 * r[1] * sina) / a2_3on2 + (2.0 * rx2 * r[1] * cosa_m1) / a2_2)
        + p[2] * (-(r[0] * r[1] * sina) / a2_3on2 - (r[2] * cosa_m1) / a2 + (r[0] * r[1] * cosa) / a2 + (rx2 * r[2] * sina) / a2_3on2 + (2.0 * rx2 * r[2] * cosa_m1) / a2_2);
    /*
           /   2                                           2       \
           | rx  ry sin(sqrt(#2))   ry sin(sqrt(#2))   2 rx  ry #1 |
        px | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                          2                        2    \
                | ry rz sin(sqrt(#2))   rx #1   ry rz cos(sqrt(#2))   rx ry  sin(sqrt(#2))   2 rx ry  #1 |
           + py | ------------------- - ----- - ------------------- + -------------------- + ----------- |
                |          3/2            #2             #2                     3/2                2     |
                \        #2                                                   #2                 #2      /
        
                /                   2                   2
                | sin(sqrt(#2))   ry  cos(sqrt(#2))   ry  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + pz | ------------- + ----------------- - ----------------- + ----------------------
                |    sqrt(#2)             #2                  3/2                    3/2
                \                                           #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(0, 1) = p[0] * ((rx2 * r[1] * sina) / a2_3on2 - (r[1] * sina) / a + (2.0 * rx2 * r[1] * cosa_m1) / a2_2)
        + p[1] * ((r[1] * r[2] * sina) / a2_3on2 - (r[0] * cosa_m1) / a2 - (r[1] * r[2] * cosa) / a2 + (r[0] * ry2 * sina) / a2_3on2 + (2.0 * r[0] * ry2 * cosa_m1) / a2_2)
        + p[2] * ((sina) / a + (ry2 * cosa) / a2 - (ry2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /   2                                           2       \
           | rx  rz sin(sqrt(#2))   rz sin(sqrt(#2))   2 rx  rz #1 |
        px | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                            2                        2    \
                |   ry rz sin(sqrt(#2))   rx #1   ry rz cos(sqrt(#2))   rx rz  sin(sqrt(#2))   2 rx rz  #1 |
           + pz | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \           #2                                                  #2                 #2      /
        
                /     2                                   2
                |   rz  cos(sqrt(#2))   sin(sqrt(#2))   rz  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + py | - ----------------- - ------------- + ----------------- + ----------------------
                |           #2             sqrt(#2)             3/2                    3/2
                \                                             #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(0, 2) = p[0] * ((rx2 * r[2] * sina) / a2_3on2 - (r[2] * sina) / a + (2.0 * rx2 * r[2] * cosa_m1) / a2_2)
        + p[2] * (-(r[1] * r[2] * sina) / a2_3on2 - (r[0] * cosa_m1) / a2 + (r[1] * r[2] * cosa) / a2 + (r[0] * rz2 * sina) / a2_3on2 + (2.0 * r[0] * rz2 * cosa_m1) / a2_2)
        + p[1] * (-(rz2 * cosa) / a2 - (sina) / a + (rz2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /      2                                           2    \
           | rx ry  sin(sqrt(#2))   rx sin(sqrt(#2))   2 rx ry  #1 |
        py | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                         2                        2       \
                |   rx rz sin(sqrt(#2))   ry #1   rx rz cos(sqrt(#2))   rx  ry sin(sqrt(#2))   2 rx  ry #1 |
           + px | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \           #2                                                  #2                 #2      /
        
                /     2                                   2
                |   rx  cos(sqrt(#2))   sin(sqrt(#2))   rx  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + pz | - ----------------- - ------------- + ----------------- + ----------------------
                |           #2             sqrt(#2)             3/2                    3/2
                \                                             #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(1, 0) = p[1] * ((r[0] * ry2 * sina) / a2_3on2 - (r[0] * sina) / a + (2.0 * r[0] * ry2 * cosa_m1) / a2_2)
        + p[0] * (-(r[0] * r[2] * sina) / a2_3on2 - (r[1] * cosa_m1) / a2 + (r[0] * r[2] * cosa) / a2 + (rx2 * r[1] * sina) / a2_3on2 + (2.0 * rx2 * r[1] * cosa_m1) / a2_2)
        + p[2] * (-(rx2 * cosa) / a2 - (sina) / a + (rx2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /   3                     3                                 \
           | ry  sin(sqrt(#2))   2 ry  #1   ry sin(sqrt(#2))   2 ry #1 |
        py | ----------------- + -------- - ---------------- - ------- |
           |         3/2              2         sqrt(#2)          #2   |
           \       #2               #2                                 /
        
                /                                                            2                        2    \
                |   ry rz sin(sqrt(#2))   rx #1   ry rz cos(sqrt(#2))   rx ry  sin(sqrt(#2))   2 rx ry  #1 |
           + px | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \           #2                                                  #2                 #2      /
        
                /                                                       2                        2       \
                | rx ry sin(sqrt(#2))   rz #1   rx ry cos(sqrt(#2))   ry  rz sin(sqrt(#2))   2 ry  rz #1 |
           + pz | ------------------- - ----- - ------------------- + -------------------- + ----------- |
                |          3/2            #2             #2                     3/2                2     |
                \        #2                                                   #2                 #2      /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(1, 1) = p[1] * ((ry3 * sina) / a2_3on2 + (2.0 * ry3 * cosa_m1) / a2_2 - (r[1] * sina) / a - (2.0 * r[1] * cosa_m1) / a2)
        + p[0] * (-(r[1] * r[2] * sina) / a2_3on2 - (r[0] * cosa_m1) / a2 + (r[1] * r[2] * cosa) / a2 + (r[0] * ry2 * sina) / a2_3on2 + (2.0 * r[0] * ry2 * cosa_m1) / a2_2)
        + p[2] * ((r[0] * r[1] * sina) / a2_3on2 - (r[2] * cosa_m1) / a2 - (r[0] * r[1] * cosa) / a2 + (ry2 * r[2] * sina) / a2_3on2 + (2.0 * ry2 * r[2] * cosa_m1) / a2_2);
    /*
           /   2                                           2       \
           | ry  rz sin(sqrt(#2))   rz sin(sqrt(#2))   2 ry  rz #1 |
        py | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                          2                        2    \
                | rx rz sin(sqrt(#2))   ry #1   rx rz cos(sqrt(#2))   ry rz  sin(sqrt(#2))   2 ry rz  #1 |
           + pz | ------------------- - ----- - ------------------- + -------------------- + ----------- |
                |          3/2            #2             #2                     3/2                2     |
                \        #2                                                   #2                 #2      /
        
                /                   2                   2
                | sin(sqrt(#2))   rz  cos(sqrt(#2))   rz  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + px | ------------- + ----------------- - ----------------- + ----------------------
                |    sqrt(#2)             #2                  3/2                    3/2
                \                                           #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(1, 2) = p[1] * ((ry2 * r[2] * sina) / a2_3on2 - (r[2] * sina) / a + (2.0 * ry2 * r[2] * cosa_m1) / a2_2)
        + p[2] * ((r[0] * r[2] * sina) / a2_3on2 - (r[1] * cosa_m1) / a2 - (r[0] * r[2] * cosa) / a2 + (r[1] * rz2 * sina) / a2_3on2 + (2.0 * r[1] * rz2 * cosa_m1) / a2_2)
        + p[0] * ((sina) / a + (rz2 * cosa) / a2 - (rz2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /      2                                           2    \
           | rx rz  sin(sqrt(#2))   rx sin(sqrt(#2))   2 rx rz  #1 |
        pz | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                       2                        2       \
                | rx ry sin(sqrt(#2))   rz #1   rx ry cos(sqrt(#2))   rx  rz sin(sqrt(#2))   2 rx  rz #1 |
           + px | ------------------- - ----- - ------------------- + -------------------- + ----------- |
                |          3/2            #2             #2                     3/2                2     |
                \        #2                                                   #2                 #2      /
        
                /                   2                   2
                | sin(sqrt(#2))   rx  cos(sqrt(#2))   rx  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + py | ------------- + ----------------- - ----------------- + ----------------------
                |    sqrt(#2)             #2                  3/2                    3/2
                \                                           #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(2, 0) = p[2] * ((r[0] * rz2 * sina) / a2_3on2 - (r[0] * sina) / a + (2.0 * r[0] * rz2 * cosa_m1) / a2_2)
        + p[0] * ((r[0] * r[1] * sina) / a2_3on2 - (r[2] * cosa_m1) / a2 - (r[0] * r[1] * cosa) / a2 + (rx2 * r[2] * sina) / a2_3on2 + (2.0 * rx2 * r[2] * cosa_m1) / a2_2)
        + p[1] * ((sina) / a + (rx2 * cosa) / a2 - (rx2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /      2                                           2    \
           | ry rz  sin(sqrt(#2))   ry sin(sqrt(#2))   2 ry rz  #1 |
        pz | -------------------- - ---------------- + ----------- |
           |           3/2              sqrt(#2)             2     |
           \         #2                                    #2      /
        
                /                                                         2                        2       \
                |   rx ry sin(sqrt(#2))   rz #1   rx ry cos(sqrt(#2))   ry  rz sin(sqrt(#2))   2 ry  rz #1 |
           + py | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \          #2                                                   #2                 #2      /
        
                /     2                                   2
                |   ry  cos(sqrt(#2))   sin(sqrt(#2))   ry  sin(sqrt(#2))   rx ry rz sin(sqrt(#2))
           + px | - ----------------- - ------------- + ----------------- + ----------------------
                |           #2             sqrt(#2)             3/2                    3/2
                \                                             #2                     #2
        
                           \
             2 rx ry rz #1 |
           + ------------- |
                    2      |
                  #2       /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(2, 1) = p[2] * ((r[1] * rz2 * sina) / a2_3on2 - (r[1] * sina) / a + (2.0 * r[1] * rz2 * cosa_m1) / a2_2)
        + p[1] * (-(r[0] * r[1] * sina) / a2_3on2 - (r[2] * cosa_m1) / a2 + (r[0] * r[1] * cosa) / a2 + (ry2 * r[2] * sina) / a2_3on2 + (2.0 * ry2 * r[2] * cosa_m1) / a2_2)
        + p[0] * (-(ry2 * cosa) / a2 - (sina) / a + (ry2 * sina) / a2_3on2 + (rxryrz * sina) / a2_3on2 + (2.0 * rxryrz * cosa_m1) / a2_2);
    /*
           /   3                     3                                 \
           | rz  sin(sqrt(#2))   2 rz  #1   rz sin(sqrt(#2))   2 rz #1 |
        pz | ----------------- + -------- - ---------------- - ------- |
           |         3/2              2         sqrt(#2)          #2   |
           \       #2               #2                                 /
        
                /                                                          2                        2    \
                | ry rz sin(sqrt(#2))   rx #1   ry rz cos(sqrt(#2))   rx rz  sin(sqrt(#2))   2 rx rz  #1 |
           + px | ------------------- - ----- - ------------------- + -------------------- + ----------- |
                |          3/2            #2             #2                     3/2                2     |
                \        #2                                                   #2                 #2      /
        
                /                                                            2                        2    \
                |   rx rz sin(sqrt(#2))   ry #1   rx rz cos(sqrt(#2))   ry rz  sin(sqrt(#2))   2 ry rz  #1 |
           + py | - ------------------- - ----- + ------------------- + -------------------- + ----------- |
                |            3/2            #2             #2                     3/2                2     |
                \          #2                                                   #2                 #2      /
        
        where
        
           #1 == cos(sqrt(#2)) - 1
        
                   2     2     2
           #2 == rx  + ry  + rz
    */
    dG_dx(2, 2) = p[2] * ((rz3 * sina) / a2_3on2 + (2.0 * rz3 * cosa_m1) / a2_2 - (r[2] * sina) / a - (2.0 * r[2] * cosa_m1) / a2)
        + p[0] * ((r[1] * r[2] * sina) / a2_3on2 - (r[0] * cosa_m1) / a2 - (r[1] * r[2] * cosa) / a2 + (r[0] * rz2 * sina) / a2_3on2 + (2.0 * r[0] * rz2 * cosa_m1) / a2_2)
        + p[1] * (-(r[0] * r[2] * sina) / a2_3on2 - (r[1] * cosa_m1) / a2 + (r[0] * r[2] * cosa) / a2 + (r[1] * rz2 * sina) / a2_3on2 + (2.0 * r[1] * rz2 * cosa_m1) / a2_2);
    dG_dx.template block<3, 3>(0, 3) = Eigen::Matrix<Scalar, 3, 3>::Identity();
    return 2.0 * dG_dx.transpose() * G;
}

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> point_to_plane_jacobian(const Eigen::Matrix<Scalar, 3, 1>& t,
        const Eigen::Matrix<Scalar, 3, 1>& r, const Eigen::Matrix<Scalar, 3, 1>& p,
        const Eigen::Matrix<Scalar, 3, 1>& q, const Eigen::Matrix<Scalar, 3, 1>& n) {
    // Setup
    const Scalar tx = t[0];
    const Scalar ty = t[1];
    const Scalar tz = t[2];
    const Scalar rx = r[0];
    const Scalar ry = r[1];
    const Scalar rz = r[2];
    const Scalar px = p[0];
    const Scalar py = p[1];
    const Scalar pz = p[2];
    const Scalar qx = q[0];
    const Scalar qy = q[1];
    const Scalar qz = q[2];
    const Scalar nx = n[0];
    const Scalar ny = n[1];
    const Scalar nz = n[2];

    const Scalar a = r.norm();
    const Scalar cosa = std::cos(a);
    const Scalar cosa_m1 = cosa - 1.0;
    const Scalar sina = std::sin(a);
    const Scalar a2 = r.squaredNorm();
    const Scalar a2_3on2 = a * a2;
    const Scalar a2_2 = a2 * a2;
    const Scalar rxryrz = rx * ry * rz;
    const Scalar rx2 = rx * rx;
    const Scalar ry2 = ry * ry;
    const Scalar rz2 = rz * rz;
    const Scalar rx3 = rx * rx2;
    const Scalar ry3 = ry * ry2;
    const Scalar rz3 = rz * rz2;
    
    // Compute directly from matlab equations
    Eigen::Matrix<Scalar, 6, 1> J;
    J(0, 0) = (2*nx*(px*((rx3*sina)/a2_3on2 + (2*rx3*cosa_m1)/a2_2 - (rx*sina)/a - (2*rx*cosa_m1)/a2) + py*((rx*rz*sina)/a2_3on2 - (rx*rz*cosa)/a2 - (ry*cosa_m1)/a2 + (rx2*ry*sina)/a2_3on2 + (2*rx2*ry*cosa_m1)/a2_2) + pz*((rx*ry*cosa)/a2 - (rz*cosa_m1)/a2 - (rx*ry*sina)/a2_3on2 + (rx2*rz*sina)/a2_3on2 + (2*rx2*rz*cosa_m1)/a2_2)) + 2*ny*(py*((rx*ry2*sina)/a2_3on2 - (rx*sina)/a + (2*rx*ry2*cosa_m1)/a2_2) + px*((rx*rz*cosa)/a2 - (ry*cosa_m1)/a2 - (rx*rz*sina)/a2_3on2 + (rx2*ry*sina)/a2_3on2 + (2*rx2*ry*cosa_m1)/a2_2) + pz*((rx2*sina)/a2_3on2 - sina/a - (rx2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)) + 2*nz*(pz*((rx*rz2*sina)/a2_3on2 - (rx*sina)/a + (2*rx*rz2*cosa_m1)/a2_2) + px*((rx*ry*sina)/a2_3on2 - (rx*ry*cosa)/a2 - (rz*cosa_m1)/a2 + (rx2*rz*sina)/a2_3on2 + (2*rx2*rz*cosa_m1)/a2_2) + py*(sina/a - (rx2*sina)/a2_3on2 + (rx2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)))*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    J(1, 0) = (2*ny*(py*((ry3*sina)/a2_3on2 + (2*ry3*cosa_m1)/a2_2 - (ry*sina)/a - (2*ry*cosa_m1)/a2) + px*((ry*rz*cosa)/a2 - (rx*cosa_m1)/a2 - (ry*rz*sina)/a2_3on2 + (rx*ry2*sina)/a2_3on2 + (2*rx*ry2*cosa_m1)/a2_2) + pz*((rx*ry*sina)/a2_3on2 - (rx*ry*cosa)/a2 - (rz*cosa_m1)/a2 + (ry2*rz*sina)/a2_3on2 + (2*ry2*rz*cosa_m1)/a2_2)) + 2*nx*(px*((rx2*ry*sina)/a2_3on2 - (ry*sina)/a + (2*rx2*ry*cosa_m1)/a2_2) + py*((ry*rz*sina)/a2_3on2 - (ry*rz*cosa)/a2 - (rx*cosa_m1)/a2 + (rx*ry2*sina)/a2_3on2 + (2*rx*ry2*cosa_m1)/a2_2) + pz*(sina/a - (ry2*sina)/a2_3on2 + (ry2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)) + 2*nz*(pz*((ry*rz2*sina)/a2_3on2 - (ry*sina)/a + (2*ry*rz2*cosa_m1)/a2_2) + py*((rx*ry*cosa)/a2 - (rz*cosa_m1)/a2 - (rx*ry*sina)/a2_3on2 + (ry2*rz*sina)/a2_3on2 + (2*ry2*rz*cosa_m1)/a2_2) + px*((ry2*sina)/a2_3on2 - sina/a - (ry2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)))*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    J(2, 0) = (2*nz*(pz*((rz3*sina)/a2_3on2 + (2*rz3*cosa_m1)/a2_2 - (rz*sina)/a - (2*rz*cosa_m1)/a2) + px*((ry*rz*sina)/a2_3on2 - (ry*rz*cosa)/a2 - (rx*cosa_m1)/a2 + (rx*rz2*sina)/a2_3on2 + (2*rx*rz2*cosa_m1)/a2_2) + py*((rx*rz*cosa)/a2 - (ry*cosa_m1)/a2 - (rx*rz*sina)/a2_3on2 + (ry*rz2*sina)/a2_3on2 + (2*ry*rz2*cosa_m1)/a2_2)) + 2*nx*(px*((rx2*rz*sina)/a2_3on2 - (rz*sina)/a + (2*rx2*rz*cosa_m1)/a2_2) + pz*((ry*rz*cosa)/a2 - (rx*cosa_m1)/a2 - (ry*rz*sina)/a2_3on2 + (rx*rz2*sina)/a2_3on2 + (2*rx*rz2*cosa_m1)/a2_2) + py*((rz2*sina)/a2_3on2 - sina/a - (rz2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)) + 2*ny*(py*((ry2*rz*sina)/a2_3on2 - (rz*sina)/a + (2*ry2*rz*cosa_m1)/a2_2) + pz*((rx*rz*sina)/a2_3on2 - (rx*rz*cosa)/a2 - (ry*cosa_m1)/a2 + (ry*rz2*sina)/a2_3on2 + (2*ry*rz2*cosa_m1)/a2_2) + px*(sina/a - (rz2*sina)/a2_3on2 + (rz2*cosa)/a2 + (rxryrz*sina)/a2_3on2 + (2*rxryrz*cosa_m1)/a2_2)))*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    J(3, 0) = 2*nx*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    J(4, 0) = 2*ny*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    J(5, 0) = 2*nz*(nx*(tx - qx + px*(cosa - (rx2*cosa_m1)/a2) - py*((rz*sina)/a + (rx*ry*cosa_m1)/a2) + pz*((ry*sina)/a - (rx*rz*cosa_m1)/a2)) + ny*(ty - qy + py*(cosa - (ry2*cosa_m1)/a2) + px*((rz*sina)/a - (rx*ry*cosa_m1)/a2) - pz*((rx*sina)/a + (ry*rz*cosa_m1)/a2)) + nz*(tz - qz + pz*(cosa - (rz2*cosa_m1)/a2) - px*((ry*sina)/a + (rx*rz*cosa_m1)/a2) + py*((rx*sina)/a - (ry*rz*cosa_m1)/a2)));
    return J;
}

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 1> point_to_point_jacobian(typename pcl::Registration<PointIn, PointOut>& registration,
        int& count) {
    // Setup
    const Eigen::Matrix4f T = registration.getFinalTransformation();
    const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    const Eigen::AngleAxisf angleaxis{R};
    const float a = angleaxis.angle();
    const float sina = std::sin(angleaxis.angle());
    const float cosa = std::cos(angleaxis.angle());
    const Eigen::Vector3f r = angleaxis.axis() * a;
    const Eigen::Vector3f t{T.block<3, 1>(0, 3)};

    const auto indices = registration.getIndices();
    const double max_correspondence_distance_squared = std::pow(registration.getMaxCorrespondenceDistance(), 2.0);
    const auto target_cloud = registration.getInputTarget();
    const auto source_cloud = registration.getInputSource();
    count = 0;
    Eigen::Matrix<float, 6, 1> jacobian = Eigen::Matrix<float, 6, 1>::Zero();
    // ith index is index of point in target that corresponds to source point i
    for (std::size_t i = 0; i < indices->size(); ++i) {
        const auto& source_point = (*source_cloud)[i];
        const auto& target_point = (*target_cloud)[(*indices)[i]];
        const float sqrEucDist = pcl::squaredEuclideanDistance(source_point, target_point);
        if (sqrEucDist <= max_correspondence_distance_squared) {
            const Eigen::Vector3f p{source_point.x, source_point.y, source_point.z};
            const Eigen::Vector3f q{target_point.x, target_point.y, target_point.z};
            jacobian += point_to_point_jacobian(R, t, r, sina, cosa, p, q);
            ++count;
        }
    }
    return jacobian;
}

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 1> point_to_plane_jacobian(typename pcl::Registration<PointIn, PointOut>& registration,
        int& count) {
    // Setup
    const Eigen::Matrix4f T = registration.getFinalTransformation();
    const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
    const Eigen::AngleAxisf angleaxis{R};
    const float a = angleaxis.angle();
    const Eigen::Vector3f r = angleaxis.axis() * a;
    const Eigen::Vector3f t{T.block<3, 1>(0, 3)};

    const auto indices = registration.getIndices();
    const double max_correspondence_distance_squared = std::pow(registration.getMaxCorrespondenceDistance(), 2.0);
    const auto target_cloud = registration.getInputTarget();
    const auto source_cloud = registration.getInputSource();
    count = 0;
    Eigen::Matrix<float, 6, 1> jacobian = Eigen::Matrix<float, 6, 1>::Zero();
    // ith index is index of point in target that corresponds to source point i
    for (std::size_t i = 0; i < indices->size(); ++i) {
        const auto& source_point = (*source_cloud)[i];
        const auto& target_point = (*target_cloud)[(*indices)[i]];
        const float sqrEucDist = pcl::squaredEuclideanDistance(source_point, target_point);
        if (sqrEucDist <= max_correspondence_distance_squared) {
            const Eigen::Vector3f p{source_point.x, source_point.y, source_point.z};
            const Eigen::Vector3f q{target_point.x, target_point.y, target_point.z};
            const Eigen::Vector3f n{target_point.normal_x, target_point.normal_y, target_point.normal_z};
            jacobian += point_to_plane_jacobian(t, r, p, q, n);
            ++count;
        }
    }
    return jacobian;
}

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 6> point_to_point_jacobian_matrix(typename pcl::Registration<PointIn, PointOut>& registration,
        int& count)
{
    const Eigen::Matrix<float, 6, 1> jacobian = point_to_point_jacobian(registration, count);
    const Eigen::Matrix<float, 6, 6> jacobian_matrix{Eigen::DiagonalMatrix<float, 6>(jacobian)};
    return jacobian_matrix;
}

template<typename PointIn, typename PointOut>
Eigen::Matrix<float, 6, 6> point_to_plane_jacobian_matrix(typename pcl::Registration<PointIn, PointOut>& registration,
        int& count)
{
    const Eigen::Matrix<float, 6, 1> jacobian = point_to_plane_jacobian(registration, count);
    const Eigen::Matrix<float, 6, 6> jacobian_matrix{Eigen::DiagonalMatrix<float, 6>(jacobian)};
    return jacobian_matrix;
}

}

#endif
