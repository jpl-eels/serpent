namespace serpent {

template<typename Model, typename Scalar>
Eigen::Matrix<double, 6, 6> censi_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double point_variance, int& correspondence_count) {
    // Setup
    eigen_ext::PrecomputedTransformComponents<double> tf{registration.getFinalTransformation().template cast<double>()};
    const auto source_cloud = registration.getInputSource();
    const auto target_cloud = registration.getInputTarget();

    // Compute correspondences
    const auto correspondences = pct::compute_registration_correspondences(registration);
    correspondence_count = correspondences->size();

    // Interate over correspondences to build half d2F_dx2 (hessian) and d2F_dzdx
    Eigen::Matrix<double, 6, 6> half_d2F_dx2 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, Eigen::Dynamic> half_d2F_dzdx(6, 6 * correspondences->size());
    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        const auto& correspondence = (*correspondences)[i];
        const typename Model::PointSource& source_point = source_cloud->at(correspondence.index_query);
        const typename Model::PointTarget& target_point = target_cloud->at(correspondence.index_match);
        Eigen::Matrix<double, 6, 6> half_d2F_dx2_i, half_d2F_dzdx_i;
        Model::process_correspondence(source_point, target_point, tf, half_d2F_dx2_i, half_d2F_dzdx_i);
        half_d2F_dx2 += half_d2F_dx2_i;
        half_d2F_dzdx.block(0, 6 * i, 6, 6) = half_d2F_dzdx_i;
    }
    const Eigen::Matrix<double, 6, 6> half_d2F_dx2_inv = half_d2F_dx2.inverse();
    // Use brackets to ensure that the Nx6K * 6KxN multiplication occurs first.
    // The point_variance multiplication is also moved outside the centre, since it is a scalar product in this case.
    return point_variance * half_d2F_dx2_inv * (half_d2F_dzdx * half_d2F_dzdx.transpose()) * half_d2F_dx2_inv;
}

// template<typename Model, typename Scalar>
template<typename Model, typename Scalar,
        typename std::enable_if_t<!(std::is_same<typename Model::PointSource, PointNormalCovariance>::value &&
                           std::is_same<typename Model::PointTarget, PointNormalCovariance>::value), int> = 0>
Eigen::Matrix<double, 6, 6> censi_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        int& correspondence_count) {
    throw std::runtime_error(
            "censi_covariance(registration) requires PointSource == PointTarget == serpent::PointNormalCovariance");
}

// template<typename Model, typename Scalar>
template<typename Model, typename Scalar,
        typename std::enable_if_t<std::is_same<typename Model::PointSource, PointNormalCovariance>::value &&
                         std::is_same<typename Model::PointTarget, PointNormalCovariance>::value, int> = 0>
Eigen::Matrix<double, 6, 6> censi_covariance(
        typename pcl::Registration<PointNormalCovariance, PointNormalCovariance, Scalar>& registration,
        int& correspondence_count) {
    // Setup
    eigen_ext::PrecomputedTransformComponents<double> tf{registration.getFinalTransformation().template cast<double>()};
    const auto source_cloud = registration.getInputSource();
    const auto target_cloud = registration.getInputTarget();

    // Compute correspondences
    const auto correspondences = pct::compute_registration_correspondences(registration);
    correspondence_count = correspondences->size();

    auto fill_sparse_matrix = [](Eigen::SparseMatrix<double>& matrix, const int index,
                                      const PointNormalCovariance& point) {
        matrix.insert(index, index) = point.covariance_xx;
        matrix.insert(index, index + 1) = point.covariance_xy;
        matrix.insert(index, index + 2) = point.covariance_xz;
        matrix.insert(index + 1, index) = point.covariance_xy;
        matrix.insert(index + 1, index + 1) = point.covariance_yy;
        matrix.insert(index + 1, index + 2) = point.covariance_yz;
        matrix.insert(index + 2, index) = point.covariance_xz;
        matrix.insert(index + 2, index + 1) = point.covariance_yz;
        matrix.insert(index + 2, index + 2) = point.covariance_zz;
    };

    // Interate over correspondences to build half d2F_dx2 (hessian) and d2F_dzdx
    Eigen::Matrix<double, 6, 6> half_d2F_dx2 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, Eigen::Dynamic> half_d2F_dzdx(6, 6 * correspondences->size());
    Eigen::SparseMatrix<double> Sigma_z(6 * correspondences->size(), 6 * correspondences->size());
    Sigma_z.reserve(Eigen::VectorXi::Constant(6 * correspondences->size(), 3));  // reserve 3 non-zero per column
    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        const auto& correspondence = (*correspondences)[i];
        const PointNormalCovariance& source_point = source_cloud->at(correspondence.index_query);
        const PointNormalCovariance& target_point = target_cloud->at(correspondence.index_match);
        Eigen::Matrix<double, 6, 6> half_d2F_dx2_i, half_d2F_dzdx_i;
        Model::process_correspondence(source_point, target_point, tf, half_d2F_dx2_i, half_d2F_dzdx_i);
        half_d2F_dx2 += half_d2F_dx2_i;
        half_d2F_dzdx.block(0, 6 * i, 6, 6) = half_d2F_dzdx_i;
        fill_sparse_matrix(Sigma_z, i * 6, source_point);
        fill_sparse_matrix(Sigma_z, i * 6 + 3, target_point);
    }
    const Eigen::Matrix<double, 6, 6> half_d2F_dx2_inv = half_d2F_dx2.inverse();
    // Use brackets to ensure that the (sparse) Nx6K * 6Kx6K * 6KxN multiplication occurs first.
    return half_d2F_dx2_inv * (half_d2F_dzdx * Sigma_z * half_d2F_dzdx.transpose()) * half_d2F_dx2_inv;
}

template<typename Model, typename Scalar>
Eigen::Matrix<double, 6, 6> lls_covariance(
        typename pcl::Registration<typename Model::PointSource, typename Model::PointTarget, Scalar>& registration,
        const double point_variance, int& correspondence_count) {
    // Setup
    eigen_ext::PrecomputedTransformComponents<double> tf{registration.getFinalTransformation().template cast<double>()};
    const auto source_cloud = registration.getInputSource();
    const auto target_cloud = registration.getInputTarget();

    // Compute correspondences
    const auto correspondences = pct::compute_registration_correspondences(registration);
    correspondence_count = correspondences->size();

    // Interate over correspondences to build hessian and d2F_dzdx
    Eigen::Matrix<double, 6, 6> half_d2F_dx2 = Eigen::Matrix<double, 6, 6>::Zero();
    for (std::size_t i = 0; i < correspondences->size(); ++i) {
        const auto& correspondence = (*correspondences)[i];
        const typename Model::PointSource& source_point = source_cloud->at(correspondence.index_query);
        const typename Model::PointTarget& target_point = target_cloud->at(correspondence.index_match);
        Eigen::Matrix<double, 6, 6> half_d2F_dx2_i;
        Model::process_correspondence(source_point, target_point, tf, half_d2F_dx2_i);
        half_d2F_dx2 += half_d2F_dx2_i;
    }
    const Eigen::Matrix<double, 6, 6> half_d2F_dx2_inv = half_d2F_dx2.inverse();
    return point_variance * half_d2F_dx2_inv;
}

template<typename PointSource, typename PointTarget>
bool PointToPointIcpNonlinearModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2, Eigen::Matrix<double, 6, 6>& half_d2F_dzdx) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q);
    half_d2F_dzdx = compute_half_d2F_dzdx(tf, p, q);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPointIcpLinearisedModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2, Eigen::Matrix<double, 6, 6>& half_d2F_dzdx) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q);
    half_d2F_dzdx = compute_half_d2F_dzdx(tf, p, q);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPointIcpNonlinearModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPointIcpLinearisedModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPlaneIcpNonlinearModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2, Eigen::Matrix<double, 6, 6>& half_d2F_dzdx) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q, n);
    half_d2F_dzdx = compute_half_d2F_dzdx(tf, p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPlaneIcpLinearisedModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2, Eigen::Matrix<double, 6, 6>& half_d2F_dzdx) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q, n);
    half_d2F_dzdx = compute_half_d2F_dzdx(tf, p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPlaneIcpNonlinearModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget>
bool PointToPlaneIcpLinearisedModel<PointSource, PointTarget>::process_correspondence(const PointSource& source_point,
        const PointTarget& target_point, const eigen_ext::PrecomputedTransformComponents<double>& tf,
        Eigen::Matrix<double, 6, 6>& half_d2F_dx2) {
    const Eigen::Vector3d p = Eigen::Vector3f{source_point.x, source_point.y, source_point.z}.cast<double>();
    const Eigen::Vector3d q = Eigen::Vector3f{target_point.x, target_point.y, target_point.z}.cast<double>();
    const Eigen::Vector3d n =
            Eigen::Vector3f{target_point.normal_x, target_point.normal_y, target_point.normal_z}.cast<double>();
    half_d2F_dx2 = compute_half_d2F_dx2(tf, p, q, n);
    return true;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPointIcpNonlinearModel<PointSource, PointTarget>::compute_half_d2F_dx2(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx3
    const double ry2 = ry * ry;                // ry3
    const double rz2 = rz * rz;                // rz3
    const double rxry = rx * ry;               // rxry
    const double rxrz = rx * rz;               // rxrz
    const double ryrz = ry * rz;               // ryrz
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double a5 = a4 * tf.a();             // a5
    const double a6 = a5 * tf.a();             // a6
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;  // cam1

    Eigen::Matrix<double, 6, 6> half_d2F_dx2;
    half_d2F_dx2(0, 0) =
            (((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
                     (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                   a * rx2ry * sa)) /
                             a4 +
                     (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((2 * pz *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * px *
                                    (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                    a4 +
                            (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 +
            (((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                            a * rx2rz * sa)) /
                             a4 +
                     (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) /
                             a4 -
                     (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((2 * px *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                                    a4 +
                            (2 * py *
                                    (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 +
            (((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                            a * rx2ry * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                   a * rx2rz * sa)) /
                             a4 -
                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * py *
                             (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * rx2ry * sa)) /
                                    a4 +
                            (2 * pz *
                                    (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                            a2 * ca * rxry + a * rx2rz * sa)) /
                                    a4 -
                            (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4)) /
                    2 +
            (((2 * py *
                      (8 * rx3ry - 6 * a2 * rxry + a3 * rz * sa - 8 * ca * rx3ry - a4 * ca * rz + 6 * a2 * ca * rxry -
                              5 * a * rx3ry * sa + 3 * a3 * rxry * sa - 3 * a * rx2rz * sa + a2 * ca * rx3ry +
                              3 * a2 * ca * rx2rz + a3 * rx2rz * sa)) /
                             a6 +
                     (2 * pz *
                             (8 * rx3rz - 6 * a2 * rxrz - a3 * ry * sa - 8 * ca * rx3rz + a4 * ca * ry +
                                     6 * a2 * ca * rxrz + 3 * a * rx2ry * sa - 5 * a * rx3rz * sa + 3 * a3 * rxrz * sa -
                                     3 * a2 * ca * rx2ry + a2 * ca * rx3rz - a3 * rx2ry * sa)) /
                             a6 -
                     (2 * px * (a2 - rx2) *
                             (2 * a2 * ca + a3 * sa - 8 * ca * rx2 - 2 * a2 + 8 * rx2 - 5 * a * rx2 * sa +
                                     a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (3 * a3 * rx * sa - 3 * a * rx3 * sa - 2 * a2 * ryrz + 8 * rx2ryrz + 3 * a2 * ca * rx3 +
                              a3 * rx3 * sa - 3 * a4 * ca * rx + 2 * a2 * ca * ryrz + a3 * ryrz * sa -
                              8 * ca * rx2ryrz + a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * py *
                             (8 * rx2ry2 - a5 * sa - 2 * a2 * ry2 - a4 * ca * rx2 + 2 * a2 * ca * ry2 + a3 * rx2 * sa +
                                     a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa)) /
                             a6 +
                     (2 * px *
                             (8 * rx3ry - 6 * a2 * rxry - a3 * rz * sa - 8 * ca * rx3ry + a4 * ca * rz +
                                     6 * a2 * ca * rxry - 5 * a * rx3ry * sa + 3 * a3 * rxry * sa + 3 * a * rx2rz * sa +
                                     a2 * ca * rx3ry - 3 * a2 * ca * rx2rz - a3 * rx2rz * sa)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (8 * rx2rz2 - a5 * sa - 2 * a2 * rz2 - a4 * ca * rx2 + 2 * a2 * ca * rz2 + a3 * rx2 * sa +
                              a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa)) /
                             a6 -
                     (2 * py *
                             (2 * a2 * ryrz - 3 * a * rx3 * sa + 3 * a3 * rx * sa - 8 * rx2ryrz + 3 * a2 * ca * rx3 +
                                     a3 * rx3 * sa - 3 * a4 * ca * rx - 2 * a2 * ca * ryrz - a3 * ryrz * sa +
                                     8 * ca * rx2ryrz - a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * px *
                             (8 * rx3rz - 6 * a2 * rxrz + a3 * ry * sa - 8 * ca * rx3rz - a4 * ca * ry +
                                     6 * a2 * ca * rxrz - 3 * a * rx2ry * sa - 5 * a * rx3rz * sa + 3 * a3 * rxrz * sa +
                                     3 * a2 * ca * rx2ry + a2 * ca * rx3rz + a3 * rx2ry * sa)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(0, 1) =
            (((2 * py *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                   a * rxry2 * sa)) /
                                    a4 +
                            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                          a * rxryrz * sa)) /
                                    a4 -
                            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                            a * rxry2 * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                   a * ry2rz * sa)) /
                             a4 -
                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * pz *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * px *
                                    (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                    a4 +
                            (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((2 * px *
                      (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                              a * rx2rz * sa)) /
                             a4 +
                     (2 * py *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                                    a4 -
                            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((2 * py *
                      (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry - 2 * a2 * ca * rxrz -
                              3 * a * rx2ry * sa - a3 * rxrz * sa + 8 * ca * rxry2rz + 3 * a2 * ca * rx2ry +
                              a3 * rx2ry * sa - a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                             a6 -
                     (2 * px *
                             (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx + 2 * a2 * ca * ryrz -
                                     3 * a * rxry2 * sa + a3 * ryrz * sa - 8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 +
                                     a3 * rxry2 * sa + a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * pz * rxry * (a4 * ca - a3 * sa + 8 * ca * rz2 - 8 * rz2 + 5 * a * rz2 * sa - a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2) +
            (((2 * py *
                      (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * ry2 +
                              a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * pz *
                             (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx - 2 * a2 * ca * ryrz -
                                     3 * a * rxry2 * sa - a3 * ryrz * sa + 8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 +
                                     a3 * rxry2 * sa - a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * px * rxry *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 - 5 * a * rx2 * sa +
                                     a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * px *
                      (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * ry2 +
                              a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * pz *
                             (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry + 2 * a2 * ca * rxrz -
                                     3 * a * rx2ry * sa + a3 * rxrz * sa - 8 * ca * rxry2rz + 3 * a2 * ca * rx2ry +
                                     a3 * rx2ry * sa + a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * py * rxry *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 - 5 * a * ry2 * sa +
                                     a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(0, 2) =
            (((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                            a * rxrz2 * sa)) /
                             a4 +
                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                   a * ryrz2 * sa)) /
                             a4 -
                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * px *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                                    a4 +
                            (2 * py *
                                    (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((2 * py *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                                    a4 -
                            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                          a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((2 * pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * px *
                             (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * rx2ry * sa)) /
                             a4 +
                     (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                   a * ryrz2 * sa)) /
                                    a4 +
                            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                          a * rxryrz * sa)) /
                                    a4 -
                            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((2 * px *
                      (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx - 2 * a2 * ca * ryrz -
                              3 * a * rxrz2 * sa - a3 * ryrz * sa + 8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 +
                              a3 * rxrz2 * sa - a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                             a6 -
                     (2 * pz *
                             (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz + 2 * a2 * ca * rxry +
                                     a3 * rxry * sa - 3 * a * rx2rz * sa - 8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz +
                                     a3 * rx2rz * sa + a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * py * rxrz * (a4 * ca - a3 * sa + 8 * ca * ry2 - 8 * ry2 + 5 * a * ry2 * sa - a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * rz2 +
                              a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * py *
                             (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx + 2 * a2 * ca * ryrz -
                                     3 * a * rxrz2 * sa + a3 * ryrz * sa - 8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 +
                                     a3 * rxrz2 * sa + a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * px * rxrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 - 5 * a * rx2 * sa +
                                     a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * px *
                      (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * rz2 +
                              a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * py *
                             (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz - 2 * a2 * ca * rxry -
                                     a3 * rxry * sa - 3 * a * rx2rz * sa + 8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz +
                                     a3 * rx2rz * sa - a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * pz * rxrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 - 5 * a * rz2 * sa +
                                     a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(0, 3) = (py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                       a * rx2ry * sa)) /
                                 a4 +
                         (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                       a * rx2rz * sa)) /
                                 a4 -
                         (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(0, 4) =
            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                          a * rx2ry * sa)) /
                    a4 -
            (pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4;
    half_d2F_dx2(0, 5) =
            (px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                          a * rx2rz * sa)) /
                    a4 +
            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4;
    half_d2F_dx2(1, 0) =
            (((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                            a * rx2ry * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                   a * rx2rz * sa)) /
                             a4 -
                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * py *
                             (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                     a * rxry2 * sa)) /
                                    a4 +
                            (2 * pz *
                                    (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((2 * px *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                                    a4 -
                            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                          a2 * ca * rxrz + a * rx2ry * sa)) /
                                    a4 +
                            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                            a * rx2rz * sa)) /
                             a4 +
                     (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) /
                             a4 -
                     (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((2 * px *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * py *
                                    (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                            a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((2 * py *
                      (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry - 2 * a2 * ca * rxrz -
                              3 * a * rx2ry * sa - a3 * rxrz * sa + 8 * ca * rxry2rz + 3 * a2 * ca * rx2ry +
                              a3 * rx2ry * sa - a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                             a6 -
                     (2 * px *
                             (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx + 2 * a2 * ca * ryrz -
                                     3 * a * rxry2 * sa + a3 * ryrz * sa - 8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 +
                                     a3 * rxry2 * sa + a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * pz * rxry * (a4 * ca - a3 * sa + 8 * ca * rz2 - 8 * rz2 + 5 * a * rz2 * sa - a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2) +
            (((2 * py *
                      (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * ry2 +
                              a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * pz *
                             (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx - 2 * a2 * ca * ryrz -
                                     3 * a * rxry2 * sa - a3 * ryrz * sa + 8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 +
                                     a3 * rxry2 * sa - a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * px * rxry *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 - 5 * a * rx2 * sa +
                                     a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * px *
                      (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * ry2 +
                              a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * pz *
                             (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry + 2 * a2 * ca * rxrz -
                                     3 * a * rx2ry * sa + a3 * rxrz * sa - 8 * ca * rxry2rz + 3 * a2 * ca * rx2ry +
                                     a3 * rx2ry * sa + a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * py * rxry *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 - 5 * a * ry2 * sa +
                                     a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(1, 1) =
            (((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                            a * rxry2 * sa)) /
                             a4 +
                     (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) /
                             a4 -
                     (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((2 * py *
                             (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                     a * rxry2 * sa)) /
                                    a4 +
                            (2 * pz *
                                    (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 +
            (((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
                     (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                   a * ry2rz * sa)) /
                             a4 +
                     (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((2 * px *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * py *
                                    (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                            a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 +
            (((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                            a * rxry2 * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                   a * ry2rz * sa)) /
                             a4 -
                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * px *
                             (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                     a * rxry2 * sa)) /
                                    a4 +
                            (2 * pz *
                                    (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                            a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 -
                            (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4)) /
                    2 +
            (((2 * px *
                      (8 * rxry3 - 6 * a2 * rxry - a3 * rz * sa - 8 * ca * rxry3 + a4 * ca * rz + 6 * a2 * ca * rxry -
                              5 * a * rxry3 * sa + 3 * a3 * rxry * sa + 3 * a * ry2rz * sa + a2 * ca * rxry3 -
                              3 * a2 * ca * ry2rz - a3 * ry2rz * sa)) /
                             a6 +
                     (2 * pz *
                             (8 * ry3rz - 6 * a2 * ryrz + a3 * rx * sa - 8 * ca * ry3rz - a4 * ca * rx +
                                     6 * a2 * ca * ryrz - 3 * a * rxry2 * sa - 5 * a * ry3rz * sa + 3 * a3 * ryrz * sa +
                                     3 * a2 * ca * rxry2 + a2 * ca * ry3rz + a3 * rxry2 * sa)) /
                             a6 -
                     (2 * py * (a2 - ry2) *
                             (2 * a2 * ca + a3 * sa - 8 * ca * ry2 - 2 * a2 + 8 * ry2 - 5 * a * ry2 * sa +
                                     a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * px *
                      (8 * rx2ry2 - a5 * sa - 2 * a2 * rx2 + 2 * a2 * ca * rx2 - a4 * ca * ry2 + a3 * rx2 * sa +
                              a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa)) /
                             a6 -
                     (2 * pz *
                             (2 * a2 * rxrz - 3 * a * ry3 * sa + 3 * a3 * ry * sa - 8 * rxry2rz + 3 * a2 * ca * ry3 +
                                     a3 * ry3 * sa - 3 * a4 * ca * ry - 2 * a2 * ca * rxrz - a3 * rxrz * sa +
                                     8 * ca * rxry2rz - a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * py *
                             (8 * rxry3 - 6 * a2 * rxry + a3 * rz * sa - 8 * ca * rxry3 - a4 * ca * rz +
                                     6 * a2 * ca * rxry - 5 * a * rxry3 * sa + 3 * a3 * rxry * sa - 3 * a * ry2rz * sa +
                                     a2 * ca * rxry3 + 3 * a2 * ca * ry2rz + a3 * ry2rz * sa)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * px *
                      (3 * a3 * ry * sa - 3 * a * ry3 * sa - 2 * a2 * rxrz + 8 * rxry2rz + 3 * a2 * ca * ry3 +
                              a3 * ry3 * sa - 3 * a4 * ca * ry + 2 * a2 * ca * rxrz + a3 * rxrz * sa -
                              8 * ca * rxry2rz + a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * pz *
                             (8 * ry2rz2 - a5 * sa - 2 * a2 * rz2 - a4 * ca * ry2 + 2 * a2 * ca * rz2 + a3 * ry2 * sa +
                                     a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa)) /
                             a6 +
                     (2 * py *
                             (8 * ry3rz - 6 * a2 * ryrz - a3 * rx * sa - 8 * ca * ry3rz + a4 * ca * rx +
                                     6 * a2 * ca * ryrz + 3 * a * rxry2 * sa - 5 * a * ry3rz * sa + 3 * a3 * ryrz * sa -
                                     3 * a2 * ca * rxry2 + a2 * ca * ry3rz - a3 * rxry2 * sa)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(1, 2) =
            (((2 * px *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                   a * ryrz2 * sa)) /
                                    a4 +
                            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                          a * rxryrz * sa)) /
                                    a4 -
                            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                            a * rxrz2 * sa)) /
                             a4 +
                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                   a * ryrz2 * sa)) /
                             a4 -
                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * px *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * py *
                                    (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                            a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((2 * py *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                                    a4 -
                            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                          a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((2 * pz *
                      (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz - 2 * a2 * ca * rxry - a3 * rxry * sa -
                              3 * a * ry2rz * sa + 8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz + a3 * ry2rz * sa -
                              a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                             a6 -
                     (2 * py *
                             (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry + 2 * a2 * ca * rxrz +
                                     a3 * rxrz * sa - 3 * a * ryrz2 * sa - 8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 +
                                     a3 * ryrz2 * sa + a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * px * ryrz * (a4 * ca - a3 * sa + 8 * ca * rx2 - 8 * rx2 + 5 * a * rx2 * sa - a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 + 2 * a2 * ca * rz2 +
                              a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * px *
                             (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry - 2 * a2 * ca * rxrz -
                                     a3 * rxrz * sa - 3 * a * ryrz2 * sa + 8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 +
                                     a3 * ryrz2 * sa - a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * py * ryrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 - 5 * a * ry2 * sa +
                                     a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * py *
                      (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 + 2 * a2 * ca * rz2 +
                              a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * px *
                             (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz + 2 * a2 * ca * rxry +
                                     a3 * rxry * sa - 3 * a * ry2rz * sa - 8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz +
                                     a3 * ry2rz * sa + a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * pz * ryrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 - 5 * a * rz2 * sa +
                                     a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(1, 3) =
            (py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                          a * rxry2 * sa)) /
                    a4 +
            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4;
    half_d2F_dx2(1, 4) = (px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                       a * rxry2 * sa)) /
                                 a4 +
                         (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                       a * ry2rz * sa)) /
                                 a4 -
                         (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(1, 5) =
            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                          a * ry2rz * sa)) /
                    a4 -
            (px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4;
    half_d2F_dx2(2, 0) =
            (((2 * px *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * py *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                   a * rx2rz * sa)) /
                                    a4 +
                            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                          a * rxryrz * sa)) /
                                    a4 -
                            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                            a * rx2ry * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                   a * rx2rz * sa)) /
                             a4 -
                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * py *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz *
                                    (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
                     (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                   a * rx2ry * sa)) /
                             a4 +
                     (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((2 * pz *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                                    a4 +
                            (2 * px *
                                    (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((2 * px *
                      (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx - 2 * a2 * ca * ryrz -
                              3 * a * rxrz2 * sa - a3 * ryrz * sa + 8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 +
                              a3 * rxrz2 * sa - a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                             a6 -
                     (2 * pz *
                             (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz + 2 * a2 * ca * rxry +
                                     a3 * rxry * sa - 3 * a * rx2rz * sa - 8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz +
                                     a3 * rx2rz * sa + a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * py * rxrz * (a4 * ca - a3 * sa + 8 * ca * ry2 - 8 * ry2 + 5 * a * ry2 * sa - a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * rz2 +
                              a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * py *
                             (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx + 2 * a2 * ca * ryrz -
                                     3 * a * rxrz2 * sa + a3 * ryrz * sa - 8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 +
                                     a3 * rxrz2 * sa + a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                             a6 +
                     (2 * px * rxrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 - 5 * a * rx2 * sa +
                                     a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * px *
                      (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 + 2 * a2 * ca * rz2 +
                              a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * py *
                             (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz - 2 * a2 * ca * rxry -
                                     a3 * rxry * sa - 3 * a * rx2rz * sa + 8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz +
                                     a3 * rx2rz * sa - a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * pz * rxrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 - 5 * a * rz2 * sa +
                                     a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(2, 1) =
            (((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                            a * rxry2 * sa)) /
                             a4 +
                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                   a * ry2rz * sa)) /
                             a4 -
                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * pz *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                                    a4 +
                            (2 * px *
                                    (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 -
            (((2 * px *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * py *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                                    a4 -
                            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    2 -
            (((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                            a * rxry2 * sa)) /
                             a4 +
                     (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) /
                             a4 -
                     (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((2 * py *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz *
                                    (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 -
            (((2 * pz *
                      (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz - 2 * a2 * ca * rxry - a3 * rxry * sa -
                              3 * a * ry2rz * sa + 8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz + a3 * ry2rz * sa -
                              a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                             a6 -
                     (2 * py *
                             (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry + 2 * a2 * ca * rxrz +
                                     a3 * rxrz * sa - 3 * a * ryrz2 * sa - 8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 +
                                     a3 * ryrz2 * sa + a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * px * ryrz * (a4 * ca - a3 * sa + 8 * ca * rx2 - 8 * rx2 + 5 * a * rx2 * sa - a2 * ca * rx2)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * pz *
                      (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 + 2 * a2 * ca * rz2 +
                              a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa -
                              3 * a * rxryrz * sa + 3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                             a6 -
                     (2 * px *
                             (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry - 2 * a2 * ca * rxrz -
                                     a3 * rxrz * sa - 3 * a * ryrz2 * sa + 8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 +
                                     a3 * ryrz2 * sa - a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                             a6 +
                     (2 * py * ryrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 - 5 * a * ry2 * sa +
                                     a2 * ca * ry2)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2) +
            (((2 * py *
                      (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 + 2 * a2 * ca * rz2 +
                              a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa +
                              3 * a * rxryrz * sa - 3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                             a6 +
                     (2 * px *
                             (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz + 2 * a2 * ca * rxry +
                                     a3 * rxry * sa - 3 * a * ry2rz * sa - 8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz +
                                     a3 * ry2rz * sa + a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * pz * ryrz *
                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 - 5 * a * rz2 * sa +
                                     a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(2, 2) =
            (((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
                     (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                   a * rxrz2 * sa)) /
                             a4 +
                     (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((2 * py *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz *
                                    (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    2 +
            (((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                            a * ryrz2 * sa)) /
                             a4 +
                     (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) /
                             a4 -
                     (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((2 * pz *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                                    a4 +
                            (2 * px *
                                    (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    2 +
            (((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                            a * rxrz2 * sa)) /
                             a4 +
                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                   a * ryrz2 * sa)) /
                             a4 -
                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((2 * px *
                             (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                     a * rxrz2 * sa)) /
                                    a4 +
                            (2 * py *
                                    (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                            a2 * ca * rxrz + a * ryrz2 * sa)) /
                                    a4 -
                            (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4)) /
                    2 +
            (((2 * px *
                      (8 * rxrz3 - 6 * a2 * rxrz + a3 * ry * sa - 8 * ca * rxrz3 - a4 * ca * ry + 6 * a2 * ca * rxrz -
                              5 * a * rxrz3 * sa + 3 * a3 * rxrz * sa - 3 * a * ryrz2 * sa + a2 * ca * rxrz3 +
                              3 * a2 * ca * ryrz2 + a3 * ryrz2 * sa)) /
                             a6 +
                     (2 * py *
                             (8 * ryrz3 - 6 * a2 * ryrz - a3 * rx * sa - 8 * ca * ryrz3 + a4 * ca * rx +
                                     6 * a2 * ca * ryrz + 3 * a * rxrz2 * sa - 5 * a * ryrz3 * sa + 3 * a3 * ryrz * sa -
                                     3 * a2 * ca * rxrz2 + a2 * ca * ryrz3 - a3 * rxrz2 * sa)) /
                             a6 -
                     (2 * pz * (a2 - rz2) *
                             (2 * a2 * ca + a3 * sa - 8 * ca * rz2 - 2 * a2 + 8 * rz2 - 5 * a * rz2 * sa +
                                     a2 * ca * rz2)) /
                             a6) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    (2 * a2) +
            (((2 * py *
                      (3 * a3 * rz * sa - 3 * a * rz3 * sa - 2 * a2 * rxry + 8 * rxryrz2 + 3 * a2 * ca * rz3 +
                              a3 * rz3 * sa - 3 * a4 * ca * rz + 2 * a2 * ca * rxry + a3 * rxry * sa -
                              8 * ca * rxryrz2 + a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * px *
                             (8 * rx2rz2 - a5 * sa - 2 * a2 * rx2 + 2 * a2 * ca * rx2 - a4 * ca * rz2 + a3 * rx2 * sa +
                                     a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa)) /
                             a6 +
                     (2 * pz *
                             (8 * rxrz3 - 6 * a2 * rxrz - a3 * ry * sa - 8 * ca * rxrz3 + a4 * ca * ry +
                                     6 * a2 * ca * rxrz - 5 * a * rxrz3 * sa + 3 * a3 * rxrz * sa + 3 * a * ryrz2 * sa +
                                     a2 * ca * rxrz3 - 3 * a2 * ca * ryrz2 - a3 * ryrz2 * sa)) /
                             a6) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    (2 * a2) +
            (((2 * py *
                      (8 * ry2rz2 - a5 * sa - 2 * a2 * ry2 + 2 * a2 * ca * ry2 - a4 * ca * rz2 + a3 * ry2 * sa +
                              a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa)) /
                             a6 -
                     (2 * px *
                             (2 * a2 * rxry - 3 * a * rz3 * sa + 3 * a3 * rz * sa - 8 * rxryrz2 + 3 * a2 * ca * rz3 +
                                     a3 * rz3 * sa - 3 * a4 * ca * rz - 2 * a2 * ca * rxry - a3 * rxry * sa +
                                     8 * ca * rxryrz2 - a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                             a6 +
                     (2 * pz *
                             (8 * ryrz3 - 6 * a2 * ryrz + a3 * rx * sa - 8 * ca * ryrz3 - a4 * ca * rx +
                                     6 * a2 * ca * ryrz - 3 * a * rxrz2 * sa - 5 * a * ryrz3 * sa + 3 * a3 * ryrz * sa +
                                     3 * a2 * ca * rxrz2 + a2 * ca * ryrz3 + a3 * rxrz2 * sa)) /
                             a6) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    (2 * a2);
    half_d2F_dx2(2, 3) =
            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                          a * rxrz2 * sa)) /
                    a4 -
            (py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4;
    half_d2F_dx2(2, 4) =
            (pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                          a * ryrz2 * sa)) /
                    a4 +
            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4;
    half_d2F_dx2(2, 5) = (px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                       a * rxrz2 * sa)) /
                                 a4 +
                         (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                       a * ryrz2 * sa)) /
                                 a4 -
                         (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(3, 0) = (py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                       a * rx2ry * sa)) /
                                 a4 +
                         (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                       a * rx2rz * sa)) /
                                 a4 -
                         (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(3, 1) =
            (py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                          a * rxry2 * sa)) /
                    a4 +
            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4;
    half_d2F_dx2(3, 2) =
            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                          a * rxrz2 * sa)) /
                    a4 -
            (py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4;
    half_d2F_dx2(3, 3) = 1;
    half_d2F_dx2(3, 4) = 0;
    half_d2F_dx2(3, 5) = 0;
    half_d2F_dx2(4, 0) =
            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                          a * rx2ry * sa)) /
                    a4 -
            (pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4;
    half_d2F_dx2(4, 1) = (px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                       a * rxry2 * sa)) /
                                 a4 +
                         (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                       a * ry2rz * sa)) /
                                 a4 -
                         (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(4, 2) =
            (pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                          a * ryrz2 * sa)) /
                    a4 +
            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4;
    half_d2F_dx2(4, 3) = 0;
    half_d2F_dx2(4, 4) = 1;
    half_d2F_dx2(4, 5) = 0;
    half_d2F_dx2(5, 0) =
            (px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                          a * rx2rz * sa)) /
                    a4 +
            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4;
    half_d2F_dx2(5, 1) =
            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                          a * ry2rz * sa)) /
                    a4 -
            (px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4;
    half_d2F_dx2(5, 2) = (px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                       a * rxrz2 * sa)) /
                                 a4 +
                         (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                       a * ryrz2 * sa)) /
                                 a4 -
                         (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4;
    half_d2F_dx2(5, 3) = 0;
    half_d2F_dx2(5, 4) = 0;
    half_d2F_dx2(5, 5) = 1;
    return half_d2F_dx2;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPointIcpNonlinearModel<PointSource, PointTarget>::compute_half_d2F_dzdx(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double rx2ry = rx * rx * ry;   // rx2ry
    const double rx2rz = rx * rx * rz;   // rx2rz
    const double rxry2 = rx * ry * ry;   // rxry2
    const double ry2rz = ry * ry * rz;   // ry2rz
    const double rxrz2 = rx * rz * rz;   // rxrz2
    const double ryrz2 = ry * rz * rz;   // ryrz2
    const double rxryrz = rx * ry * rz;  // rxryrz
    const double rx3 = rx * rx * rx;     // rx3
    const double ry3 = ry * ry * ry;     // ry3
    const double rz3 = rz * rz * rz;     // rz3
    const double rx2 = rx * rx;          // rx2
    const double ry2 = ry * ry;          // ry2
    const double rz2 = rz * rz;          // rz2
    const double rxry = rx * ry;         // rxry
    const double rxrz = rx * rz;         // rxrz
    const double ryrz = ry * rz;         // ryrz
    const double a = tf.a();             // a
    const double a2 = a * tf.a();        // a2
    const double a3 = a2 * tf.a();       // a3
    const double a4 = a3 * tf.a();       // a4
    const double a5 = a4 * tf.a();       // a5
    const double a6 = a5 * tf.a();       // a6
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;  // cam1

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dzdx;
    half_d2F_dzdx(0, 0) =
            ((a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz + a * rx2ry * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 -
            (((2 * px *
                      (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                              a * rx2rz * sa)) /
                             a4 +
                     (2 * py *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    (a * ry * sa - rxrz + ca * rxrz)) /
                    (2 * a2) -
            (((2 * pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * px *
                             (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * rx2ry * sa)) /
                             a4 +
                     (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    (rxry + a * rz * sa - ca * rxry)) /
                    (2 * a2) +
            ((a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry + a * rx2rz * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 +
            (((2 * py *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    (a2 * ca - ca * rx2 + rx2)) /
                    (2 * a2) -
            (rx * (a2 - rx2) * (2 * ca + a * sa - 2) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6;
    half_d2F_dzdx(0, 1) =
            (((2 * px *
                      (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                              a * rx2rz * sa)) /
                             a4 +
                     (2 * py *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    (ryrz + a * rx * sa - ca * ryrz)) /
                    (2 * a2) -
            (((2 * py *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    (a * rz * sa - rxry + ca * rxry)) /
                    (2 * a2) +
            ((a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz + a * rx2ry * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 +
            ((a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 -
            ((a2 * ca - ca * ry2 + ry2) *
                    ((2 * pz *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * px *
                                    (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                    a4 +
                            (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    (2 * a2) -
            (rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6;
    half_d2F_dzdx(0, 2) =
            (((2 * py *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    (rxrz + a * ry * sa - ca * rxrz)) /
                    (2 * a2) +
            (((2 * pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * px *
                             (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * rx2ry * sa)) /
                             a4 +
                     (2 * py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    (a * rx * sa - ryrz + ca * ryrz)) /
                    (2 * a2) +
            ((a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry + a * rx2rz * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 -
            ((a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 +
            ((a2 * ca - ca * rz2 + rz2) *
                    ((2 * px *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                                    a4 +
                            (2 * py *
                                    (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    (2 * a2) -
            (rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6;
    half_d2F_dzdx(0, 3) = (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4 -
                          (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                        a * rx2rz * sa)) /
                                  a4 -
                          (py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                        a * rx2ry * sa)) /
                                  a4;
    half_d2F_dzdx(0, 4) =
            (pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                          a * rx2ry * sa)) /
                    a4 +
            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4;
    half_d2F_dzdx(0, 5) =
            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4 -
            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                          a * rx2rz * sa)) /
                    a4;
    half_d2F_dzdx(1, 0) =
            (((2 * px *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    (rxry + a * rz * sa - ca * rxry)) /
                    (2 * a2) +
            (((2 * px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * py *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 +
                     (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    (a * ry * sa - rxrz + ca * rxrz)) /
                    (2 * a2) +
            ((a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz + a * rxry2 * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 -
            ((a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 +
            ((a2 * ca - ca * rx2 + rx2) *
                    ((2 * py *
                             (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                     a * rxry2 * sa)) /
                                    a4 +
                            (2 * pz *
                                    (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    (2 * a2) -
            (ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6;
    half_d2F_dzdx(1, 1) =
            ((a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz + a * rxry2 * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 -
            (((2 * px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * py *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 +
                     (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    (ryrz + a * rx * sa - ca * ryrz)) /
                    (2 * a2) -
            (((2 * py *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    (a * rz * sa - rxry + ca * rxry)) /
                    (2 * a2) +
            ((a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry + a * ry2rz * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 +
            (((2 * px *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    (a2 * ca - ca * ry2 + ry2)) /
                    (2 * a2) -
            (ry * (a2 - ry2) * (2 * ca + a * sa - 2) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6;
    half_d2F_dzdx(1, 2) =
            (((2 * py *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    (rxrz + a * ry * sa - ca * rxrz)) /
                    (2 * a2) -
            (((2 * px *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * pz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    (a * rx * sa - ryrz + ca * ryrz)) /
                    (2 * a2) +
            ((a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry + a * ry2rz * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 +
            ((a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 -
            ((a2 * ca - ca * rz2 + rz2) *
                    ((2 * px *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * py *
                                    (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                            a2 * ca * rxry + a * ry2rz * sa)) /
                                    a4 +
                            (2 * pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4)) /
                    (2 * a2) -
            (ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6;
    half_d2F_dzdx(1, 3) =
            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4 -
            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                          a * rxry2 * sa)) /
                    a4;
    half_d2F_dzdx(1, 4) = (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4 -
                          (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                        a * ry2rz * sa)) /
                                  a4 -
                          (px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                        a * rxry2 * sa)) /
                                  a4;
    half_d2F_dzdx(1, 5) =
            (px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                          a * ry2rz * sa)) /
                    a4 +
            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4;
    half_d2F_dzdx(2, 0) =
            (((2 * pz *
                      (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * ryrz2 * sa)) /
                             a4 +
                     (2 * px *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    (rxry + a * rz * sa - ca * rxry)) /
                    (2 * a2) -
            (((2 * px *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * py *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    (a * ry * sa - rxrz + ca * rxrz)) /
                    (2 * a2) +
            ((a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz + a * rxrz2 * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 +
            ((a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 -
            ((a2 * ca - ca * rx2 + rx2) *
                    ((2 * py *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                     a * rxryrz * sa)) /
                                    a4 -
                            (2 * pz *
                                    (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                    a4 +
                            (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4)) /
                    (2 * a2) -
            (rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6;
    half_d2F_dzdx(2, 1) =
            (((2 * px *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * py *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    (ryrz + a * rx * sa - ca * ryrz)) /
                    (2 * a2) +
            (((2 * py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * pz *
                             (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                     a * rxrz2 * sa)) /
                             a4 +
                     (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    (a * rz * sa - rxry + ca * rxry)) /
                    (2 * a2) +
            ((a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz + a * ryrz2 * sa) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6 -
            ((a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 +
            ((a2 * ca - ca * ry2 + ry2) *
                    ((2 * pz *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                                    a4 +
                            (2 * px *
                                    (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                            a * rxryrz * sa)) /
                                    a4 -
                            (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4)) /
                    (2 * a2) -
            (rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6;
    half_d2F_dzdx(2, 2) =
            ((a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz + a * rxrz2 * sa) *
                    (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                    a6 -
            (((2 * pz *
                      (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * ryrz2 * sa)) /
                             a4 +
                     (2 * px *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    (a * rx * sa - ryrz + ca * ryrz)) /
                    (2 * a2) -
            (((2 * py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * pz *
                             (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                     a * rxrz2 * sa)) /
                             a4 +
                     (2 * px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    (rxrz + a * ry * sa - ca * rxrz)) /
                    (2 * a2) +
            ((a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz + a * ryrz2 * sa) *
                    (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                            a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                    a6 +
            (((2 * px *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * py *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    (a2 * ca - ca * rz2 + rz2)) /
                    (2 * a2) -
            (rz * (a2 - rz2) * (2 * ca + a * sa - 2) *
                    (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                            a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                    a6;
    half_d2F_dzdx(2, 3) =
            (py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) / a4 -
            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                          a * rxrz2 * sa)) /
                    a4 +
            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4;
    half_d2F_dzdx(2, 4) =
            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4 -
            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz + a * rxryrz * sa)) / a4 -
            (pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                          a * ryrz2 * sa)) /
                    a4;
    half_d2F_dzdx(2, 5) = (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4 -
                          (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                        a * ryrz2 * sa)) /
                                  a4 -
                          (px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                                        a * rxrz2 * sa)) /
                                  a4;
    half_d2F_dzdx(3, 0) = ca - (rx2 * (cam1)) / a2;
    half_d2F_dzdx(3, 1) = -(rz * sa) / a - (rxry * (cam1)) / a2;
    half_d2F_dzdx(3, 2) = (ry * sa) / a - (rxrz * (cam1)) / a2;
    half_d2F_dzdx(3, 3) = -1;
    half_d2F_dzdx(3, 4) = 0;
    half_d2F_dzdx(3, 5) = 0;
    half_d2F_dzdx(4, 0) = (rz * sa) / a - (rxry * (cam1)) / a2;
    half_d2F_dzdx(4, 1) = ca - (ry2 * (cam1)) / a2;
    half_d2F_dzdx(4, 2) = -(rx * sa) / a - (ryrz * (cam1)) / a2;
    half_d2F_dzdx(4, 3) = 0;
    half_d2F_dzdx(4, 4) = -1;
    half_d2F_dzdx(4, 5) = 0;
    half_d2F_dzdx(5, 0) = -(ry * sa) / a - (rxrz * (cam1)) / a2;
    half_d2F_dzdx(5, 1) = (rx * sa) / a - (ryrz * (cam1)) / a2;
    half_d2F_dzdx(5, 2) = ca - (rz2 * (cam1)) / a2;
    half_d2F_dzdx(5, 3) = 0;
    half_d2F_dzdx(5, 4) = 0;
    half_d2F_dzdx(5, 5) = -1;
    return half_d2F_dzdx;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPointIcpLinearisedModel<PointSource, PointTarget>::compute_half_d2F_dx2(
        const eigen_ext::PrecomputedTransformComponents<double>&, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) {
    // Compute required quantities once
    const Eigen::Matrix3d p_skew = eigen_ext::skew_symmetric(p);

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dx2;
    half_d2F_dx2 << -p_skew * p_skew, p_skew, -p_skew, Eigen::Matrix3d::Identity();
    return half_d2F_dx2;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPointIcpLinearisedModel<PointSource, PointTarget>::compute_half_d2F_dzdx(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q) {
    // Compute required quantities once
    const Eigen::Matrix3d r_skew = eigen_ext::skew_symmetric(tf.r());
    const Eigen::Matrix3d p_skew = eigen_ext::skew_symmetric(p);

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dzdx;
    half_d2F_dzdx << p_skew * r_skew +
                             eigen_ext::skew_symmetric(Eigen::Matrix<double, 3, 1>{p.cross(tf.r()) - tf.t() + q}),
            -p_skew, r_skew + Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
    return half_d2F_dzdx;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpNonlinearModel<PointSource, PointTarget>::compute_half_d2F_dx2(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx4 = rx * rx * rx * rx;      // rx4
    const double ry4 = ry * ry * ry * ry;      // ry4
    const double rz4 = rz * rz * rz * rz;      // rz4
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx2
    const double ry2 = ry * ry;                // ry2
    const double rz2 = rz * rz;                // rz2
    const double rxry = rx * ry;               // rxry
    const double ryrz = ry * rz;               // ryrz
    const double rxrz = rx * rz;               // rxrz
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double a5 = a4 * tf.a();             // a5
    const double a6 = a5 * tf.a();             // a6
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;   // cam1
    const double nx2 = nx * nx;   // nx2
    const double ny2 = ny * ny;   // ny2
    const double nz2 = nz * nz;   // nz2
    const double nxny = nx * ny;  // nxny
    const double nynz = ny * nz;  // nynz
    const double nxnz = nx * nz;  // nxnz

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dx2;
    half_d2F_dx2(0, 0) =
            (((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                             a2 +
                     (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                   a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                             a2 +
                     (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                   a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                             a2) *
                    (2 * nx *
                                    ((py * (8 * rx3ry - 6 * a2 * rxry + a3 * rz * sa - 8 * ca * rx3ry - a4 * ca * rz +
                                                   6 * a2 * ca * rxry - 5 * a * rx3ry * sa + 3 * a3 * rxry * sa -
                                                   3 * a * rx2rz * sa + a2 * ca * rx3ry + 3 * a2 * ca * rx2rz +
                                                   a3 * rx2rz * sa)) /
                                                    a6 +
                                            (pz * (8 * rx3rz - 6 * a2 * rxrz - a3 * ry * sa - 8 * ca * rx3rz +
                                                          a4 * ca * ry + 6 * a2 * ca * rxrz + 3 * a * rx2ry * sa -
                                                          5 * a * rx3rz * sa + 3 * a3 * rxrz * sa -
                                                          3 * a2 * ca * rx2ry + a2 * ca * rx3rz - a3 * rx2ry * sa)) /
                                                    a6 -
                                            (px * (a2 - rx2) *
                                                    (2 * a2 * ca + a3 * sa - 8 * ca * rx2 - 2 * a2 + 8 * rx2 -
                                                            5 * a * rx2 * sa + a2 * ca * rx2)) /
                                                    a6) +
                            2 * ny *
                                    ((pz * (3 * a3 * rx * sa - 3 * a * rx3 * sa - 2 * a2 * ryrz + 8 * rx2ryrz +
                                                   3 * a2 * ca * rx3 + a3 * rx3 * sa - 3 * a4 * ca * rx +
                                                   2 * a2 * ca * ryrz + a3 * ryrz * sa - 8 * ca * rx2ryrz +
                                                   a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                                                    a6 +
                                            (py * (8 * rx2ry2 - a5 * sa - 2 * a2 * ry2 - a4 * ca * rx2 +
                                                          2 * a2 * ca * ry2 + a3 * rx2 * sa + a3 * ry2 * sa -
                                                          8 * ca * rx2ry2 + a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa)) /
                                                    a6 +
                                            (px * (8 * rx3ry - 6 * a2 * rxry - a3 * rz * sa - 8 * ca * rx3ry +
                                                          a4 * ca * rz + 6 * a2 * ca * rxry - 5 * a * rx3ry * sa +
                                                          3 * a3 * rxry * sa + 3 * a * rx2rz * sa + a2 * ca * rx3ry -
                                                          3 * a2 * ca * rx2rz - a3 * rx2rz * sa)) /
                                                    a6) +
                            2 * nz *
                                    ((pz * (8 * rx2rz2 - a5 * sa - 2 * a2 * rz2 - a4 * ca * rx2 + 2 * a2 * ca * rz2 +
                                                   a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 + a2 * ca * rx2rz2 -
                                                   5 * a * rx2rz2 * sa)) /
                                                    a6 -
                                            (py * (2 * a2 * ryrz - 3 * a * rx3 * sa + 3 * a3 * rx * sa - 8 * rx2ryrz +
                                                          3 * a2 * ca * rx3 + a3 * rx3 * sa - 3 * a4 * ca * rx -
                                                          2 * a2 * ca * ryrz - a3 * ryrz * sa + 8 * ca * rx2ryrz -
                                                          a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                                                    a6 +
                                            (px * (8 * rx3rz - 6 * a2 * rxrz + a3 * ry * sa - 8 * ca * rx3rz -
                                                          a4 * ca * ry + 6 * a2 * ca * rxrz - 3 * a * rx2ry * sa -
                                                          5 * a * rx3rz * sa + 3 * a3 * rxrz * sa +
                                                          3 * a2 * ca * rx2ry + a2 * ca * rx3rz + a3 * rx2ry * sa)) /
                                                    a6))) /
                    2 +
            ((2 * nx *
                             ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                             a4 -
                                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * ny *
                             (px * ry * (rx2 + ry2 + rz2) - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa -
                                     2 * pz * rxryrz - a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * px * ry * (rx2 + ry2 + rz2) -
                                     ca * pz * rx2 * (rx2 + ry2 + rz2) - a * px * rxrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa + ca * px * rxrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * (rx2 + ry2 + rz2) -
                                     2 * py * rxryrz - a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * px * rz * (rx2 + ry2 + rz2) +
                                     ca * py * rx2 * (rx2 + ry2 + rz2) + a * px * rxry * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa - ca * px * rxry * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                            nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                            nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                            2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                            a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                            a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                            2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                            2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                            ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa -
                            ca * nx * py * rxrz * (rx2 + ry2 + rz2) + ca * nx * pz * rxry * (rx2 + ry2 + rz2) +
                            ca * ny * px * rxrz * (rx2 + ry2 + rz2) - ca * nz * px * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(0, 1) =
            ((2 * nx *
                             ((py * (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * ry2 + a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 +
                                            a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (pz * (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx -
                                                   2 * a2 * ca * ryrz - 3 * a * rxry2 * sa - a3 * ryrz * sa +
                                                   8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 + a3 * rxry2 * sa -
                                                   a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                                             a6 +
                                     (px * rxry *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 -
                                                     5 * a * rx2 * sa + a2 * ca * rx2)) /
                                             a6) +
                     2 * ny *
                             ((px * (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * ry2 + a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 +
                                            a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (pz * (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry +
                                                   2 * a2 * ca * rxrz - 3 * a * rx2ry * sa + a3 * rxrz * sa -
                                                   8 * ca * rxry2rz + 3 * a2 * ca * rx2ry + a3 * rx2ry * sa +
                                                   a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                                             a6 +
                                     (py * rxry *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 -
                                                     5 * a * ry2 * sa + a2 * ca * ry2)) /
                                             a6) +
                     (2 * nz *
                             (a4 * ca * py * ry - a4 * ca * px * rx + a3 * px * rx * sa - a3 * py * ry * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * px * ryrz * (rx2 + ry2 + rz2) - 2 * py * rxrz * (rx2 + ry2 + rz2) +
                                     a3 * px * rxry2 * sa - a3 * py * rx2ry * sa +
                                     3 * ca * px * rxry2 * (rx2 + ry2 + rz2) - 3 * ca * py * rx2ry * (rx2 + ry2 + rz2) -
                                     a4 * ca * pz * rxry - 3 * a * px * rxry2 * sa + 3 * a * py * rx2ry * sa +
                                     a3 * px * ryrz * sa + a3 * py * rxrz * sa + a3 * pz * rxry * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * px * ryrz * (rx2 + ry2 + rz2) + 2 * ca * py * rxrz * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * nx *
                             ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                             a4 -
                                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * ny *
                             (px * ry * (rx2 + ry2 + rz2) - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa -
                                     2 * pz * rxryrz - a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * px * ry * (rx2 + ry2 + rz2) -
                                     ca * pz * rx2 * (rx2 + ry2 + rz2) - a * px * rxrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa + ca * px * rxrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * (rx2 + ry2 + rz2) -
                                     2 * py * rxryrz - a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * px * rz * (rx2 + ry2 + rz2) +
                                     ca * py * rx2 * (rx2 + ry2 + rz2) + a * px * rxry * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa - ca * px * rxry * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                            ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                            ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                            ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                            2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                            a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                            a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                            2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                            2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * (rx2 + ry2 + rz2) - ca * ny * px * rx * (rx2 + ry2 + rz2) -
                            2 * ca * ny * py * ry * (rx2 + ry2 + rz2) - ca * ny * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * py * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                            ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(0, 2) =
            ((2 * nx *
                             ((pz * (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * rz2 + a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 +
                                            a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (py * (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx +
                                                   2 * a2 * ca * ryrz - 3 * a * rxrz2 * sa + a3 * ryrz * sa -
                                                   8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 + a3 * rxrz2 * sa +
                                                   a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                                             a6 +
                                     (px * rxrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 -
                                                     5 * a * rx2 * sa + a2 * ca * rx2)) /
                                             a6) +
                     2 * nz *
                             ((px * (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * rz2 + a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 +
                                            a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (py * (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz -
                                                   2 * a2 * ca * rxry - a3 * rxry * sa - 3 * a * rx2rz * sa +
                                                   8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz + a3 * rx2rz * sa -
                                                   a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                                             a6 +
                                     (pz * rxrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 -
                                                     5 * a * rz2 * sa + a2 * ca * rz2)) /
                                             a6) +
                     (2 * ny *
                             (a4 * ca * px * rx - a4 * ca * pz * rz - a3 * px * rx * sa + a3 * pz * rz * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * px * ryrz * (rx2 + ry2 + rz2) - 2 * pz * rxry * (rx2 + ry2 + rz2) -
                                     a3 * px * rxrz2 * sa + a3 * pz * rx2rz * sa -
                                     3 * ca * px * rxrz2 * (rx2 + ry2 + rz2) + 3 * ca * pz * rx2rz * (rx2 + ry2 + rz2) -
                                     a4 * ca * py * rxrz + 3 * a * px * rxrz2 * sa + a3 * px * ryrz * sa +
                                     a3 * py * rxrz * sa + a3 * pz * rxry * sa - 3 * a * pz * rx2rz * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * px * ryrz * (rx2 + ry2 + rz2) + 2 * ca * pz * rxry * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * nx *
                             ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                            a2 * ca * rxrz + a * rx2ry * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                             a4 -
                                     (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * ny *
                             (px * ry * (rx2 + ry2 + rz2) - 2 * px * rx2ry - 2 * py * rxry2 - a3 * pz * sa -
                                     2 * pz * rxryrz - a3 * py * rx * sa + a * pz * rx2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * px * ry * (rx2 + ry2 + rz2) -
                                     ca * pz * rx2 * (rx2 + ry2 + rz2) - a * px * rxrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa + ca * px * rxrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * py * sa - 2 * px * rx2rz - 2 * pz * rxrz2 + px * rz * (rx2 + ry2 + rz2) -
                                     2 * py * rxryrz - a * py * rx2 * sa - a3 * pz * rx * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * px * rz * (rx2 + ry2 + rz2) +
                                     ca * py * rx2 * (rx2 + ry2 + rz2) + a * px * rxry * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa - ca * px * rxry * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                            nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                            nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                            2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                            a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                            a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                            2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                            2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * (rx2 + ry2 + rz2) - ca * nz * px * rx * (rx2 + ry2 + rz2) -
                            ca * ny * pz * ry * (rx2 + ry2 + rz2) - ca * nz * py * ry * (rx2 + ry2 + rz2) -
                            2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                            ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                            ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(0, 3) =
            (nx * (2 * nx *
                                  ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 -
                                          (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * ny *
                                  ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                          2 * nz *
                                  ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                 a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 +
                                          (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(0, 4) =
            (ny * (2 * nx *
                                  ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 -
                                          (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * ny *
                                  ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                          2 * nz *
                                  ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                 a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 +
                                          (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(0, 5) =
            (nz * (2 * nx *
                                  ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 -
                                          (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * ny *
                                  ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * rx2ry * sa)) /
                                                  a4 +
                                          (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                          2 * nz *
                                  ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                 a2 * ca * rxry + a * rx2rz * sa)) /
                                                  a4 +
                                          (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(1, 0) =
            ((2 * nx *
                             ((py * (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * ry2 + a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 +
                                            a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (pz * (2 * a2 * ryrz + a3 * rx * sa - 8 * rx2ryrz - a4 * ca * rx -
                                                   2 * a2 * ca * ryrz - 3 * a * rxry2 * sa - a3 * ryrz * sa +
                                                   8 * ca * rx2ryrz + 3 * a2 * ca * rxry2 + a3 * rxry2 * sa -
                                                   a2 * ca * rx2ryrz + 5 * a * rx2ryrz * sa)) /
                                             a6 +
                                     (px * rxry *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 -
                                                     5 * a * rx2 * sa + a2 * ca * rx2)) /
                                             a6) +
                     2 * ny *
                             ((px * (8 * rx2ry2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * ry2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * ry2 + a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 +
                                            a2 * ca * rx2ry2 - 5 * a * rx2ry2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (pz * (a3 * ry * sa - 2 * a2 * rxrz + 8 * rxry2rz - a4 * ca * ry +
                                                   2 * a2 * ca * rxrz - 3 * a * rx2ry * sa + a3 * rxrz * sa -
                                                   8 * ca * rxry2rz + 3 * a2 * ca * rx2ry + a3 * rx2ry * sa +
                                                   a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                                             a6 +
                                     (py * rxry *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 -
                                                     5 * a * ry2 * sa + a2 * ca * ry2)) /
                                             a6) +
                     (2 * nz *
                             (a4 * ca * py * ry - a4 * ca * px * rx + a3 * px * rx * sa - a3 * py * ry * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * px * ryrz * (rx2 + ry2 + rz2) - 2 * py * rxrz * (rx2 + ry2 + rz2) +
                                     a3 * px * rxry2 * sa - a3 * py * rx2ry * sa +
                                     3 * ca * px * rxry2 * (rx2 + ry2 + rz2) - 3 * ca * py * rx2ry * (rx2 + ry2 + rz2) -
                                     a4 * ca * pz * rxry - 3 * a * px * rxry2 * sa + 3 * a * py * rx2ry * sa +
                                     a3 * px * ryrz * sa + a3 * py * rxrz * sa + a3 * pz * rxry * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * px * ryrz * (rx2 + ry2 + rz2) + 2 * ca * py * rxrz * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * ny *
                             ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxry2 * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * ry2rz * sa)) /
                                             a4 -
                                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * (rx2 + ry2 + rz2) -
                                     2 * pz * rxryrz - a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * py * rx * (rx2 + ry2 + rz2) +
                                     ca * pz * ry2 * (rx2 + ry2 + rz2) + a * py * ryrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa - ca * py * ryrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * (rx2 + ry2 + rz2) - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa -
                                     2 * px * rxryrz + a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * py * rz * (rx2 + ry2 + rz2) -
                                     ca * px * ry2 * (rx2 + ry2 + rz2) - a * py * rxry * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa + ca * py * rxry * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                            nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                            nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                            2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                            a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                            a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                            2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                            2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                            ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa -
                            ca * nx * py * rxrz * (rx2 + ry2 + rz2) + ca * nx * pz * rxry * (rx2 + ry2 + rz2) +
                            ca * ny * px * rxrz * (rx2 + ry2 + rz2) - ca * nz * px * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(1, 1) =
            (((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                             a2 +
                     (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                   a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                             a2 +
                     (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                   a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                             a2) *
                    (2 * ny *
                                    ((px * (8 * rxry3 - 6 * a2 * rxry - a3 * rz * sa - 8 * ca * rxry3 + a4 * ca * rz +
                                                   6 * a2 * ca * rxry - 5 * a * rxry3 * sa + 3 * a3 * rxry * sa +
                                                   3 * a * ry2rz * sa + a2 * ca * rxry3 - 3 * a2 * ca * ry2rz -
                                                   a3 * ry2rz * sa)) /
                                                    a6 +
                                            (pz * (8 * ry3rz - 6 * a2 * ryrz + a3 * rx * sa - 8 * ca * ry3rz -
                                                          a4 * ca * rx + 6 * a2 * ca * ryrz - 3 * a * rxry2 * sa -
                                                          5 * a * ry3rz * sa + 3 * a3 * ryrz * sa +
                                                          3 * a2 * ca * rxry2 + a2 * ca * ry3rz + a3 * rxry2 * sa)) /
                                                    a6 -
                                            (py * (a2 - ry2) *
                                                    (2 * a2 * ca + a3 * sa - 8 * ca * ry2 - 2 * a2 + 8 * ry2 -
                                                            5 * a * ry2 * sa + a2 * ca * ry2)) /
                                                    a6) +
                            2 * nx *
                                    ((px * (8 * rx2ry2 - a5 * sa - 2 * a2 * rx2 + 2 * a2 * ca * rx2 - a4 * ca * ry2 +
                                                   a3 * rx2 * sa + a3 * ry2 * sa - 8 * ca * rx2ry2 + a2 * ca * rx2ry2 -
                                                   5 * a * rx2ry2 * sa)) /
                                                    a6 -
                                            (pz * (2 * a2 * rxrz - 3 * a * ry3 * sa + 3 * a3 * ry * sa - 8 * rxry2rz +
                                                          3 * a2 * ca * ry3 + a3 * ry3 * sa - 3 * a4 * ca * ry -
                                                          2 * a2 * ca * rxrz - a3 * rxrz * sa + 8 * ca * rxry2rz -
                                                          a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                                                    a6 +
                                            (py * (8 * rxry3 - 6 * a2 * rxry + a3 * rz * sa - 8 * ca * rxry3 -
                                                          a4 * ca * rz + 6 * a2 * ca * rxry - 5 * a * rxry3 * sa +
                                                          3 * a3 * rxry * sa - 3 * a * ry2rz * sa + a2 * ca * rxry3 +
                                                          3 * a2 * ca * ry2rz + a3 * ry2rz * sa)) /
                                                    a6) +
                            2 * nz *
                                    ((px * (3 * a3 * ry * sa - 3 * a * ry3 * sa - 2 * a2 * rxrz + 8 * rxry2rz +
                                                   3 * a2 * ca * ry3 + a3 * ry3 * sa - 3 * a4 * ca * ry +
                                                   2 * a2 * ca * rxrz + a3 * rxrz * sa - 8 * ca * rxry2rz +
                                                   a2 * ca * rxry2rz - 5 * a * rxry2rz * sa)) /
                                                    a6 +
                                            (pz * (8 * ry2rz2 - a5 * sa - 2 * a2 * rz2 - a4 * ca * ry2 +
                                                          2 * a2 * ca * rz2 + a3 * ry2 * sa + a3 * rz2 * sa -
                                                          8 * ca * ry2rz2 + a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa)) /
                                                    a6 +
                                            (py * (8 * ry3rz - 6 * a2 * ryrz - a3 * rx * sa - 8 * ca * ry3rz +
                                                          a4 * ca * rx + 6 * a2 * ca * ryrz + 3 * a * rxry2 * sa -
                                                          5 * a * ry3rz * sa + 3 * a3 * ryrz * sa -
                                                          3 * a2 * ca * rxry2 + a2 * ca * ry3rz - a3 * rxry2 * sa)) /
                                                    a6))) /
                    2 +
            ((2 * ny *
                             ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxry2 * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * ry2rz * sa)) /
                                             a4 -
                                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * (rx2 + ry2 + rz2) -
                                     2 * pz * rxryrz - a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * py * rx * (rx2 + ry2 + rz2) +
                                     ca * pz * ry2 * (rx2 + ry2 + rz2) + a * py * ryrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa - ca * py * ryrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * (rx2 + ry2 + rz2) - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa -
                                     2 * px * rxryrz + a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * py * rz * (rx2 + ry2 + rz2) -
                                     ca * px * ry2 * (rx2 + ry2 + rz2) - a * py * rxry * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa + ca * py * rxry * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                            ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                            ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                            ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                            2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                            a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                            a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                            2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                            2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * (rx2 + ry2 + rz2) - ca * ny * px * rx * (rx2 + ry2 + rz2) -
                            2 * ca * ny * py * ry * (rx2 + ry2 + rz2) - ca * ny * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * py * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                            ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(1, 2) =
            ((2 * ny *
                             ((pz * (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 +
                                            2 * a2 * ca * rz2 + a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 +
                                            a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (px * (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry -
                                                   2 * a2 * ca * rxrz - a3 * rxrz * sa - 3 * a * ryrz2 * sa +
                                                   8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 + a3 * ryrz2 * sa -
                                                   a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                                             a6 +
                                     (py * ryrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 -
                                                     5 * a * ry2 * sa + a2 * ca * ry2)) /
                                             a6) +
                     2 * nz *
                             ((py * (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 +
                                            2 * a2 * ca * rz2 + a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 +
                                            a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (px * (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz +
                                                   2 * a2 * ca * rxry + a3 * rxry * sa - 3 * a * ry2rz * sa -
                                                   8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz + a3 * ry2rz * sa +
                                                   a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                                             a6 +
                                     (pz * ryrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 -
                                                     5 * a * rz2 * sa + a2 * ca * rz2)) /
                                             a6) +
                     (2 * nx *
                             (a4 * ca * pz * rz - a4 * ca * py * ry + a3 * py * ry * sa - a3 * pz * rz * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * py * rxrz * (rx2 + ry2 + rz2) - 2 * pz * rxry * (rx2 + ry2 + rz2) +
                                     a3 * py * ryrz2 * sa - a3 * pz * ry2rz * sa +
                                     3 * ca * py * ryrz2 * (rx2 + ry2 + rz2) - 3 * ca * pz * ry2rz * (rx2 + ry2 + rz2) -
                                     a4 * ca * px * ryrz + a3 * px * ryrz * sa + a3 * py * rxrz * sa +
                                     a3 * pz * rxry * sa - 3 * a * py * ryrz2 * sa + 3 * a * pz * ry2rz * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * py * rxrz * (rx2 + ry2 + rz2) + 2 * ca * pz * rxry * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * ny *
                             ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                            a2 * ca * ryrz + a * rxry2 * sa)) /
                                             a4 +
                                     (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * ry2rz * sa)) /
                                             a4 -
                                     (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (a3 * pz * sa - 2 * px * rx2ry - 2 * py * rxry2 + py * rx * (rx2 + ry2 + rz2) -
                                     2 * pz * rxryrz - a3 * px * ry * sa - a * pz * ry2 * sa + 2 * ca * px * rx2ry +
                                     2 * ca * py * rxry2 - ca * py * rx * (rx2 + ry2 + rz2) +
                                     ca * pz * ry2 * (rx2 + ry2 + rz2) + a * py * ryrz * sa + 2 * ca * pz * rxryrz +
                                     a * px * rx2ry * sa + a * py * rxry2 * sa - ca * py * ryrz * (rx2 + ry2 + rz2) +
                                     a * pz * rxryrz * sa)) /
                             a4 +
                     (2 * nz *
                             (py * rz * (rx2 + ry2 + rz2) - 2 * py * ry2rz - 2 * pz * ryrz2 - a3 * px * sa -
                                     2 * px * rxryrz + a * px * ry2 * sa - a3 * pz * ry * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * py * rz * (rx2 + ry2 + rz2) -
                                     ca * px * ry2 * (rx2 + ry2 + rz2) - a * py * rxry * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa + ca * py * rxry * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                            nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                            nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                            2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                            a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                            a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                            2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                            2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * (rx2 + ry2 + rz2) - ca * nz * px * rx * (rx2 + ry2 + rz2) -
                            ca * ny * pz * ry * (rx2 + ry2 + rz2) - ca * nz * py * ry * (rx2 + ry2 + rz2) -
                            2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                            ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                            ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(1, 3) =
            (nx * (2 * ny *
                                  ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 -
                                          (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                          2 * nx *
                                  ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                          2 * nz *
                                  ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 +
                                          (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(1, 4) =
            (ny * (2 * ny *
                                  ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 -
                                          (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                          2 * nx *
                                  ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                          2 * nz *
                                  ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 +
                                          (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(1, 5) =
            (nz * (2 * ny *
                                  ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 -
                                          (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                          2 * nx *
                                  ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxry2 * sa)) /
                                                  a4 +
                                          (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                          2 * nz *
                                  ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                        a2 * ca * rxry + a * ry2rz * sa)) /
                                                  a4 +
                                          (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dx2(2, 0) =
            ((2 * nx *
                             ((pz * (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * rz2 + a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 +
                                            a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (py * (a3 * rx * sa - 2 * a2 * ryrz + 8 * rx2ryrz - a4 * ca * rx +
                                                   2 * a2 * ca * ryrz - 3 * a * rxrz2 * sa + a3 * ryrz * sa -
                                                   8 * ca * rx2ryrz + 3 * a2 * ca * rxrz2 + a3 * rxrz2 * sa +
                                                   a2 * ca * rx2ryrz - 5 * a * rx2ryrz * sa)) /
                                             a6 +
                                     (px * rxrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rx2 - 4 * a2 + 8 * rx2 -
                                                     5 * a * rx2 * sa + a2 * ca * rx2)) /
                                             a6) +
                     2 * nz *
                             ((px * (8 * rx2rz2 - a4 * ca + a4 - 2 * a2 * rx2 - 2 * a2 * rz2 + 2 * a2 * ca * rx2 +
                                            2 * a2 * ca * rz2 + a3 * rx2 * sa + a3 * rz2 * sa - 8 * ca * rx2rz2 +
                                            a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (py * (2 * a2 * rxry + a3 * rz * sa - 8 * rxryrz2 - a4 * ca * rz -
                                                   2 * a2 * ca * rxry - a3 * rxry * sa - 3 * a * rx2rz * sa +
                                                   8 * ca * rxryrz2 + 3 * a2 * ca * rx2rz + a3 * rx2rz * sa -
                                                   a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                                             a6 +
                                     (pz * rxrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 -
                                                     5 * a * rz2 * sa + a2 * ca * rz2)) /
                                             a6) +
                     (2 * ny *
                             (a4 * ca * px * rx - a4 * ca * pz * rz - a3 * px * rx * sa + a3 * pz * rz * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * px * ryrz * (rx2 + ry2 + rz2) - 2 * pz * rxry * (rx2 + ry2 + rz2) -
                                     a3 * px * rxrz2 * sa + a3 * pz * rx2rz * sa -
                                     3 * ca * px * rxrz2 * (rx2 + ry2 + rz2) + 3 * ca * pz * rx2rz * (rx2 + ry2 + rz2) -
                                     a4 * ca * py * rxrz + 3 * a * px * rxrz2 * sa + a3 * px * ryrz * sa +
                                     a3 * py * rxrz * sa + a3 * pz * rxry * sa - 3 * a * pz * rx2rz * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * px * ryrz * (rx2 + ry2 + rz2) + 2 * ca * pz * rxry * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * nz *
                             ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                             a4 +
                                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                             a4 -
                                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (pz * rx * (rx2 + ry2 + rz2) - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa -
                                     2 * py * rxryrz - a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * pz * rx * (rx2 + ry2 + rz2) -
                                     ca * py * rz2 * (rx2 + ry2 + rz2) - a * pz * ryrz * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa + ca * pz * ryrz * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * (rx2 + ry2 + rz2) -
                                     2 * px * rxryrz - a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * pz * ry * (rx2 + ry2 + rz2) +
                                     ca * px * rz2 * (rx2 + ry2 + rz2) + a * pz * rxrz * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa - ca * pz * rxrz * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                            2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                            2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                            nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                            nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                            2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                            a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                            a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                            2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                            2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                            2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                            ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa +
                            a * ny * px * rx2ry * sa + a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa +
                            a * nz * px * rx2rz * sa + a * nz * pz * rxrz2 * sa -
                            ca * nx * py * rxrz * (rx2 + ry2 + rz2) + ca * nx * pz * rxry * (rx2 + ry2 + rz2) +
                            ca * ny * px * rxrz * (rx2 + ry2 + rz2) - ca * nz * px * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * rxrz * sa - a * nx * pz * rxry * sa - a * ny * px * rxrz * sa +
                            a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz + 2 * ca * nz * py * rxryrz +
                            a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(2, 1) =
            ((2 * ny *
                             ((pz * (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 +
                                            2 * a2 * ca * rz2 + a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 +
                                            a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa - 3 * a * rxryrz * sa +
                                            3 * a2 * ca * rxryrz + a3 * rxryrz * sa)) /
                                             a6 -
                                     (px * (2 * a2 * rxrz + a3 * ry * sa - 8 * rxry2rz - a4 * ca * ry -
                                                   2 * a2 * ca * rxrz - a3 * rxrz * sa - 3 * a * ryrz2 * sa +
                                                   8 * ca * rxry2rz + 3 * a2 * ca * ryrz2 + a3 * ryrz2 * sa -
                                                   a2 * ca * rxry2rz + 5 * a * rxry2rz * sa)) /
                                             a6 +
                                     (py * ryrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * ry2 - 4 * a2 + 8 * ry2 -
                                                     5 * a * ry2 * sa + a2 * ca * ry2)) /
                                             a6) +
                     2 * nz *
                             ((py * (8 * ry2rz2 - a4 * ca + a4 - 2 * a2 * ry2 - 2 * a2 * rz2 + 2 * a2 * ca * ry2 +
                                            2 * a2 * ca * rz2 + a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 +
                                            a2 * ca * ry2rz2 - 5 * a * ry2rz2 * sa + 3 * a * rxryrz * sa -
                                            3 * a2 * ca * rxryrz - a3 * rxryrz * sa)) /
                                             a6 +
                                     (px * (a3 * rz * sa - 2 * a2 * rxry + 8 * rxryrz2 - a4 * ca * rz +
                                                   2 * a2 * ca * rxry + a3 * rxry * sa - 3 * a * ry2rz * sa -
                                                   8 * ca * rxryrz2 + 3 * a2 * ca * ry2rz + a3 * ry2rz * sa +
                                                   a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                                             a6 +
                                     (pz * ryrz *
                                             (4 * a2 * ca - a4 * ca + 3 * a3 * sa - 8 * ca * rz2 - 4 * a2 + 8 * rz2 -
                                                     5 * a * rz2 * sa + a2 * ca * rz2)) /
                                             a6) +
                     (2 * nx *
                             (a4 * ca * pz * rz - a4 * ca * py * ry + a3 * py * ry * sa - a3 * pz * rz * sa +
                                     8 * px * rx2ryrz + 8 * py * rxry2rz + 8 * pz * rxryrz2 -
                                     2 * py * rxrz * (rx2 + ry2 + rz2) - 2 * pz * rxry * (rx2 + ry2 + rz2) +
                                     a3 * py * ryrz2 * sa - a3 * pz * ry2rz * sa +
                                     3 * ca * py * ryrz2 * (rx2 + ry2 + rz2) - 3 * ca * pz * ry2rz * (rx2 + ry2 + rz2) -
                                     a4 * ca * px * ryrz + a3 * px * ryrz * sa + a3 * py * rxrz * sa +
                                     a3 * pz * rxry * sa - 3 * a * py * ryrz2 * sa + 3 * a * pz * ry2rz * sa -
                                     8 * ca * px * rx2ryrz - 8 * ca * py * rxry2rz - 8 * ca * pz * rxryrz2 +
                                     2 * ca * py * rxrz * (rx2 + ry2 + rz2) + 2 * ca * pz * rxry * (rx2 + ry2 + rz2) -
                                     5 * a * px * rx2ryrz * sa - 5 * a * py * rxry2rz * sa - 5 * a * pz * rxryrz2 * sa +
                                     ca * px * rx2ryrz * (rx2 + ry2 + rz2) + ca * py * rxry2rz * (rx2 + ry2 + rz2) +
                                     ca * pz * rxryrz2 * (rx2 + ry2 + rz2))) /
                             a6) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            ((2 * nz *
                             ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                             a4 +
                                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                             a4 -
                                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (pz * rx * (rx2 + ry2 + rz2) - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa -
                                     2 * py * rxryrz - a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * pz * rx * (rx2 + ry2 + rz2) -
                                     ca * py * rz2 * (rx2 + ry2 + rz2) - a * pz * ryrz * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa + ca * pz * ryrz * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * (rx2 + ry2 + rz2) -
                                     2 * px * rxryrz - a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * pz * ry * (rx2 + ry2 + rz2) +
                                     ca * px * rz2 * (rx2 + ry2 + rz2) + a * pz * rxrz * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa - ca * pz * rxrz * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                            2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                            2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                            ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                            ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                            ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                            2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                            a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                            a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                            2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                            2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 -
                            ca * nx * py * rx * (rx2 + ry2 + rz2) - ca * ny * px * rx * (rx2 + ry2 + rz2) -
                            2 * ca * ny * py * ry * (rx2 + ry2 + rz2) - ca * ny * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nz * py * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2ry * sa +
                            a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa + a * ny * pz * ry2rz * sa +
                            a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                            ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                            ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                            a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                            a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                            a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(2, 2) =
            (((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                            a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                             a2 +
                     (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                   a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                             a2 +
                     (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                   a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                             a2) *
                    (2 * nz *
                                    ((px * (8 * rxrz3 - 6 * a2 * rxrz + a3 * ry * sa - 8 * ca * rxrz3 - a4 * ca * ry +
                                                   6 * a2 * ca * rxrz - 5 * a * rxrz3 * sa + 3 * a3 * rxrz * sa -
                                                   3 * a * ryrz2 * sa + a2 * ca * rxrz3 + 3 * a2 * ca * ryrz2 +
                                                   a3 * ryrz2 * sa)) /
                                                    a6 +
                                            (py * (8 * ryrz3 - 6 * a2 * ryrz - a3 * rx * sa - 8 * ca * ryrz3 +
                                                          a4 * ca * rx + 6 * a2 * ca * ryrz + 3 * a * rxrz2 * sa -
                                                          5 * a * ryrz3 * sa + 3 * a3 * ryrz * sa -
                                                          3 * a2 * ca * rxrz2 + a2 * ca * ryrz3 - a3 * rxrz2 * sa)) /
                                                    a6 -
                                            (pz * (a2 - rz2) *
                                                    (2 * a2 * ca + a3 * sa - 8 * ca * rz2 - 2 * a2 + 8 * rz2 -
                                                            5 * a * rz2 * sa + a2 * ca * rz2)) /
                                                    a6) +
                            2 * nx *
                                    ((py * (3 * a3 * rz * sa - 3 * a * rz3 * sa - 2 * a2 * rxry + 8 * rxryrz2 +
                                                   3 * a2 * ca * rz3 + a3 * rz3 * sa - 3 * a4 * ca * rz +
                                                   2 * a2 * ca * rxry + a3 * rxry * sa - 8 * ca * rxryrz2 +
                                                   a2 * ca * rxryrz2 - 5 * a * rxryrz2 * sa)) /
                                                    a6 +
                                            (px * (8 * rx2rz2 - a5 * sa - 2 * a2 * rx2 + 2 * a2 * ca * rx2 -
                                                          a4 * ca * rz2 + a3 * rx2 * sa + a3 * rz2 * sa -
                                                          8 * ca * rx2rz2 + a2 * ca * rx2rz2 - 5 * a * rx2rz2 * sa)) /
                                                    a6 +
                                            (pz * (8 * rxrz3 - 6 * a2 * rxrz - a3 * ry * sa - 8 * ca * rxrz3 +
                                                          a4 * ca * ry + 6 * a2 * ca * rxrz - 5 * a * rxrz3 * sa +
                                                          3 * a3 * rxrz * sa + 3 * a * ryrz2 * sa + a2 * ca * rxrz3 -
                                                          3 * a2 * ca * ryrz2 - a3 * ryrz2 * sa)) /
                                                    a6) +
                            2 * ny *
                                    ((py * (8 * ry2rz2 - a5 * sa - 2 * a2 * ry2 + 2 * a2 * ca * ry2 - a4 * ca * rz2 +
                                                   a3 * ry2 * sa + a3 * rz2 * sa - 8 * ca * ry2rz2 + a2 * ca * ry2rz2 -
                                                   5 * a * ry2rz2 * sa)) /
                                                    a6 -
                                            (px * (2 * a2 * rxry - 3 * a * rz3 * sa + 3 * a3 * rz * sa - 8 * rxryrz2 +
                                                          3 * a2 * ca * rz3 + a3 * rz3 * sa - 3 * a4 * ca * rz -
                                                          2 * a2 * ca * rxry - a3 * rxry * sa + 8 * ca * rxryrz2 -
                                                          a2 * ca * rxryrz2 + 5 * a * rxryrz2 * sa)) /
                                                    a6 +
                                            (pz * (8 * ryrz3 - 6 * a2 * ryrz + a3 * rx * sa - 8 * ca * ryrz3 -
                                                          a4 * ca * rx + 6 * a2 * ca * ryrz - 3 * a * rxrz2 * sa -
                                                          5 * a * ryrz3 * sa + 3 * a3 * ryrz * sa +
                                                          3 * a2 * ca * rxrz2 + a2 * ca * ryrz3 + a3 * rxrz2 * sa)) /
                                                    a6))) /
                    2 +
            ((2 * nz *
                             ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                            a2 * ca * ryrz + a * rxrz2 * sa)) /
                                             a4 +
                                     (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                             a4 -
                                     (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) +
                     (2 * nx *
                             (pz * rx * (rx2 + ry2 + rz2) - 2 * px * rx2rz - 2 * pz * rxrz2 - a3 * py * sa -
                                     2 * py * rxryrz - a3 * px * rz * sa + a * py * rz2 * sa + 2 * ca * px * rx2rz +
                                     2 * ca * pz * rxrz2 - ca * pz * rx * (rx2 + ry2 + rz2) -
                                     ca * py * rz2 * (rx2 + ry2 + rz2) - a * pz * ryrz * sa + 2 * ca * py * rxryrz +
                                     a * px * rx2rz * sa + a * pz * rxrz2 * sa + ca * pz * ryrz * (rx2 + ry2 + rz2) +
                                     a * py * rxryrz * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * px * sa - 2 * py * ry2rz - 2 * pz * ryrz2 + pz * ry * (rx2 + ry2 + rz2) -
                                     2 * px * rxryrz - a * px * rz2 * sa - a3 * py * rz * sa + 2 * ca * py * ry2rz +
                                     2 * ca * pz * ryrz2 - ca * pz * ry * (rx2 + ry2 + rz2) +
                                     ca * px * rz2 * (rx2 + ry2 + rz2) + a * pz * rxrz * sa + 2 * ca * px * rxryrz +
                                     a * py * ry2rz * sa + a * pz * ryrz2 * sa - ca * pz * rxrz * (rx2 + ry2 + rz2) +
                                     a * px * rxryrz * sa)) /
                             a4) *
                    (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                            2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                            2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                            nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                            nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                            ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                            2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                            a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                            a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                            2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                            2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 -
                            ca * nx * pz * rx * (rx2 + ry2 + rz2) - ca * nz * px * rx * (rx2 + ry2 + rz2) -
                            ca * ny * pz * ry * (rx2 + ry2 + rz2) - ca * nz * py * ry * (rx2 + ry2 + rz2) -
                            2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) + a * nx * px * rx2rz * sa +
                            a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa + a * ny * py * ry2rz * sa +
                            a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                            ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                            ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                            a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                            a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                            a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
                    (2 * a4);
    half_d2F_dx2(2, 3) =
            (nx * (2 * nz *
                                  ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 -
                                          (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * nx *
                                  ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                        a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                          2 * ny *
                                  ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 +
                                          (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dx2(2, 4) =
            (ny * (2 * nz *
                                  ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 -
                                          (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * nx *
                                  ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                        a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                          2 * ny *
                                  ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 +
                                          (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dx2(2, 5) =
            (nz * (2 * nz *
                                  ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                 a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                        a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 -
                                          (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                          2 * nx *
                                  ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                 a * rxryrz * sa)) /
                                                  a4 -
                                          (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                        a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                  a4 +
                                          (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                          2 * ny *
                                  ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                 a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                  a4 +
                                          (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                                        a * rxryrz * sa)) /
                                                  a4 -
                                          (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dx2(3, 0) =
            (nx * (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                          2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                          2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                          nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                          nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                          2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                          a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                          a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                          2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                          2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                          2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa + a * ny * px * rx2ry * sa +
                          a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa + a * nz * px * rx2rz * sa +
                          a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * rxry * (rx2 + ry2 + rz2) + ca * ny * px * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rxry * (rx2 + ry2 + rz2) + a * nx * py * rxrz * sa - a * nx * pz * rxry * sa -
                          a * ny * px * rxrz * sa + a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz +
                          2 * ca * nz * py * rxryrz + a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    half_d2F_dx2(3, 1) =
            (nx * (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                          2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                          2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                          ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                          ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                          2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                          a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                          a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                          2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                          2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 - ca * nx * py * rx * (rx2 + ry2 + rz2) -
                          ca * ny * px * rx * (rx2 + ry2 + rz2) - 2 * ca * ny * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rz * (rx2 + ry2 + rz2) - ca * nz * py * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2ry * sa + a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa +
                          a * ny * pz * ry2rz * sa + a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                          ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                          a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                          a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                          a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(3, 2) =
            (nx * (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                          2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                          2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                          nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                          nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                          2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                          a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                          a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                          2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                          2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 - ca * nx * pz * rx * (rx2 + ry2 + rz2) -
                          ca * nz * px * rx * (rx2 + ry2 + rz2) - ca * ny * pz * ry * (rx2 + ry2 + rz2) -
                          ca * nz * py * ry * (rx2 + ry2 + rz2) - 2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2rz * sa + a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa +
                          a * ny * py * ry2rz * sa + a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                          ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                          a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                          a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                          a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(3, 3) = nx2;
    half_d2F_dx2(3, 4) = nx * ny;
    half_d2F_dx2(3, 5) = nx * nz;
    half_d2F_dx2(4, 0) =
            (ny * (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                          2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                          2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                          nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                          nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                          2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                          a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                          a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                          2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                          2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                          2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa + a * ny * px * rx2ry * sa +
                          a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa + a * nz * px * rx2rz * sa +
                          a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * rxry * (rx2 + ry2 + rz2) + ca * ny * px * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rxry * (rx2 + ry2 + rz2) + a * nx * py * rxrz * sa - a * nx * pz * rxry * sa -
                          a * ny * px * rxrz * sa + a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz +
                          2 * ca * nz * py * rxryrz + a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    half_d2F_dx2(4, 1) =
            (ny * (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                          2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                          2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                          ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                          ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                          2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                          a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                          a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                          2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                          2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 - ca * nx * py * rx * (rx2 + ry2 + rz2) -
                          ca * ny * px * rx * (rx2 + ry2 + rz2) - 2 * ca * ny * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rz * (rx2 + ry2 + rz2) - ca * nz * py * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2ry * sa + a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa +
                          a * ny * pz * ry2rz * sa + a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                          ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                          a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                          a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                          a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(4, 2) =
            (ny * (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                          2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                          2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                          nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                          nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                          2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                          a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                          a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                          2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                          2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 - ca * nx * pz * rx * (rx2 + ry2 + rz2) -
                          ca * nz * px * rx * (rx2 + ry2 + rz2) - ca * ny * pz * ry * (rx2 + ry2 + rz2) -
                          ca * nz * py * ry * (rx2 + ry2 + rz2) - 2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2rz * sa + a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa +
                          a * ny * py * ry2rz * sa + a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                          ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                          a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                          a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                          a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(4, 3) = nx * ny;
    half_d2F_dx2(4, 4) = ny2;
    half_d2F_dx2(4, 5) = ny * nz;
    half_d2F_dx2(5, 0) =
            (nz * (a3 * nz * py * sa - a3 * ny * pz * sa - 2 * nx * px * rx3 + 2 * ca * nx * px * rx3 -
                          2 * nx * py * rx2ry - 2 * ny * px * rx2ry - 2 * ny * py * rxry2 - 2 * nx * pz * rx2rz -
                          2 * nz * px * rx2rz - 2 * nz * pz * rxrz2 + 2 * nx * px * rx * (rx2 + ry2 + rz2) +
                          nx * py * ry * (rx2 + ry2 + rz2) + ny * px * ry * (rx2 + ry2 + rz2) +
                          nx * pz * rz * (rx2 + ry2 + rz2) + nz * px * rz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rx2 * (rx2 + ry2 + rz2) + ca * nz * py * rx2 * (rx2 + ry2 + rz2) -
                          2 * ny * pz * rxryrz - 2 * nz * py * rxryrz + a * nx * px * rx3 * sa -
                          a3 * nx * px * rx * sa - a3 * ny * py * rx * sa + a * ny * pz * rx2 * sa -
                          a * nz * py * rx2 * sa - a3 * nz * pz * rx * sa + 2 * ca * nx * py * rx2ry +
                          2 * ca * ny * px * rx2ry + 2 * ca * ny * py * rxry2 + 2 * ca * nx * pz * rx2rz +
                          2 * ca * nz * px * rx2rz + 2 * ca * nz * pz * rxrz2 -
                          2 * ca * nx * px * rx * (rx2 + ry2 + rz2) - ca * nx * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * px * ry * (rx2 + ry2 + rz2) - ca * nx * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rz * (rx2 + ry2 + rz2) + a * nx * py * rx2ry * sa + a * ny * px * rx2ry * sa +
                          a * ny * py * rxry2 * sa + a * nx * pz * rx2rz * sa + a * nz * px * rx2rz * sa +
                          a * nz * pz * rxrz2 * sa - ca * nx * py * rxrz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * rxry * (rx2 + ry2 + rz2) + ca * ny * px * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * rxry * (rx2 + ry2 + rz2) + a * nx * py * rxrz * sa - a * nx * pz * rxry * sa -
                          a * ny * px * rxrz * sa + a * nz * px * rxry * sa + 2 * ca * ny * pz * rxryrz +
                          2 * ca * nz * py * rxryrz + a * ny * pz * rxryrz * sa + a * nz * py * rxryrz * sa)) /
            a4;
    half_d2F_dx2(5, 1) =
            (nz * (a3 * nx * pz * sa - 2 * ny * py * ry3 - a3 * nz * px * sa + 2 * ca * ny * py * ry3 -
                          2 * nx * px * rx2ry - 2 * nx * py * rxry2 - 2 * ny * px * rxry2 - 2 * ny * pz * ry2rz -
                          2 * nz * py * ry2rz - 2 * nz * pz * ryrz2 + nx * py * rx * (rx2 + ry2 + rz2) +
                          ny * px * rx * (rx2 + ry2 + rz2) + 2 * ny * py * ry * (rx2 + ry2 + rz2) +
                          ny * pz * rz * (rx2 + ry2 + rz2) + nz * py * rz * (rx2 + ry2 + rz2) +
                          ca * nx * pz * ry2 * (rx2 + ry2 + rz2) - ca * nz * px * ry2 * (rx2 + ry2 + rz2) -
                          2 * nx * pz * rxryrz - 2 * nz * px * rxryrz - a3 * nx * px * ry * sa -
                          a * nx * pz * ry2 * sa + a * nz * px * ry2 * sa + a * ny * py * ry3 * sa -
                          a3 * ny * py * ry * sa - a3 * nz * pz * ry * sa + 2 * ca * nx * px * rx2ry +
                          2 * ca * nx * py * rxry2 + 2 * ca * ny * px * rxry2 + 2 * ca * ny * pz * ry2rz +
                          2 * ca * nz * py * ry2rz + 2 * ca * nz * pz * ryrz2 - ca * nx * py * rx * (rx2 + ry2 + rz2) -
                          ca * ny * px * rx * (rx2 + ry2 + rz2) - 2 * ca * ny * py * ry * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rz * (rx2 + ry2 + rz2) - ca * nz * py * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2ry * sa + a * nx * py * rxry2 * sa + a * ny * px * rxry2 * sa +
                          a * ny * pz * ry2rz * sa + a * nz * py * ry2rz * sa + a * nz * pz * ryrz2 * sa -
                          ca * nx * py * ryrz * (rx2 + ry2 + rz2) + ca * ny * px * ryrz * (rx2 + ry2 + rz2) -
                          ca * ny * pz * rxry * (rx2 + ry2 + rz2) + ca * nz * py * rxry * (rx2 + ry2 + rz2) +
                          a * nx * py * ryrz * sa - a * ny * px * ryrz * sa + a * ny * pz * rxry * sa -
                          a * nz * py * rxry * sa + 2 * ca * nx * pz * rxryrz + 2 * ca * nz * px * rxryrz +
                          a * nx * pz * rxryrz * sa + a * nz * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(5, 2) =
            (nz * (a3 * ny * px * sa - a3 * nx * py * sa - 2 * nz * pz * rz3 + 2 * ca * nz * pz * rz3 -
                          2 * nx * px * rx2rz - 2 * nx * pz * rxrz2 - 2 * nz * px * rxrz2 - 2 * ny * py * ry2rz -
                          2 * ny * pz * ryrz2 - 2 * nz * py * ryrz2 + nx * pz * rx * (rx2 + ry2 + rz2) +
                          nz * px * rx * (rx2 + ry2 + rz2) + ny * pz * ry * (rx2 + ry2 + rz2) +
                          nz * py * ry * (rx2 + ry2 + rz2) + 2 * nz * pz * rz * (rx2 + ry2 + rz2) -
                          ca * nx * py * rz2 * (rx2 + ry2 + rz2) + ca * ny * px * rz2 * (rx2 + ry2 + rz2) -
                          2 * nx * py * rxryrz - 2 * ny * px * rxryrz - a3 * nx * px * rz * sa +
                          a * nx * py * rz2 * sa - a * ny * px * rz2 * sa - a3 * ny * py * rz * sa +
                          a * nz * pz * rz3 * sa - a3 * nz * pz * rz * sa + 2 * ca * nx * px * rx2rz +
                          2 * ca * nx * pz * rxrz2 + 2 * ca * nz * px * rxrz2 + 2 * ca * ny * py * ry2rz +
                          2 * ca * ny * pz * ryrz2 + 2 * ca * nz * py * ryrz2 - ca * nx * pz * rx * (rx2 + ry2 + rz2) -
                          ca * nz * px * rx * (rx2 + ry2 + rz2) - ca * ny * pz * ry * (rx2 + ry2 + rz2) -
                          ca * nz * py * ry * (rx2 + ry2 + rz2) - 2 * ca * nz * pz * rz * (rx2 + ry2 + rz2) +
                          a * nx * px * rx2rz * sa + a * nx * pz * rxrz2 * sa + a * nz * px * rxrz2 * sa +
                          a * ny * py * ry2rz * sa + a * ny * pz * ryrz2 * sa + a * nz * py * ryrz2 * sa +
                          ca * nx * pz * ryrz * (rx2 + ry2 + rz2) - ca * ny * pz * rxrz * (rx2 + ry2 + rz2) -
                          ca * nz * px * ryrz * (rx2 + ry2 + rz2) + ca * nz * py * rxrz * (rx2 + ry2 + rz2) -
                          a * nx * pz * ryrz * sa + a * ny * pz * rxrz * sa + a * nz * px * ryrz * sa -
                          a * nz * py * rxrz * sa + 2 * ca * nx * py * rxryrz + 2 * ca * ny * px * rxryrz +
                          a * nx * py * rxryrz * sa + a * ny * px * rxryrz * sa)) /
            a4;
    half_d2F_dx2(5, 3) = nx * nz;
    half_d2F_dx2(5, 4) = ny * nz;
    half_d2F_dx2(5, 5) = nz2;
    return half_d2F_dx2;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpNonlinearModel<PointSource, PointTarget>::compute_half_d2F_dzdx(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) {
    // Aliases
    const double rx{tf.r()[0]};
    const double ry{tf.r()[1]};
    const double rz{tf.r()[2]};
    const double tx{tf.t()[0]};
    const double ty{tf.t()[1]};
    const double tz{tf.t()[2]};
    const double px{p[0]};
    const double py{p[1]};
    const double pz{p[2]};
    const double qx{q[0]};
    const double qy{q[1]};
    const double qz{q[2]};
    const double nx{n[0]};
    const double ny{n[1]};
    const double nz{n[2]};
    const double rx2ryrz = rx * rx * ry * rz;  // rx2ryrz
    const double rxry2rz = rx * ry * ry * rz;  // rxry2rz
    const double rxryrz2 = rx * ry * rz * rz;  // rxryrz2
    const double rx2ry2 = rx * rx * ry * ry;   // rx2ry2
    const double rx2rz2 = rx * rx * rz * rz;   // rx2rz2
    const double ry2rz2 = ry * ry * rz * rz;   // ry2rz2
    const double rx3ry = rx * rx * rx * ry;    // rx3ry
    const double rx3rz = rx * rx * rx * rz;    // rx3rz
    const double rxry3 = rx * ry * ry * ry;    // rxry3
    const double ry3rz = ry * ry * ry * rz;    // ry3rz
    const double rxrz3 = rx * rz * rz * rz;    // rxrz3
    const double ryrz3 = ry * rz * rz * rz;    // ryrz3
    const double rx4 = rx * rx * rx * rx;      // rx4
    const double ry4 = ry * ry * ry * ry;      // ry4
    const double rz4 = rz * rz * rz * rz;      // rz4
    const double rx2ry = rx * rx * ry;         // rx2ry
    const double rx2rz = rx * rx * rz;         // rx2rz
    const double rxry2 = rx * ry * ry;         // rxry2
    const double ry2rz = ry * ry * rz;         // ry2rz
    const double rxrz2 = rx * rz * rz;         // rxrz2
    const double ryrz2 = ry * rz * rz;         // ryrz2
    const double rxryrz = rx * ry * rz;        // rxryrz
    const double rx3 = rx * rx * rx;           // rx3
    const double ry3 = ry * ry * ry;           // ry3
    const double rz3 = rz * rz * rz;           // rz3
    const double rx2 = rx * rx;                // rx2
    const double ry2 = ry * ry;                // ry2
    const double rz2 = rz * rz;                // rz2
    const double rxry = rx * ry;               // rxry
    const double ryrz = ry * rz;               // ryrz
    const double rxrz = rx * rz;               // rxrz
    const double a = tf.a();                   // a
    const double a2 = a * tf.a();              // a2
    const double a3 = a2 * tf.a();             // a3
    const double a4 = a3 * tf.a();             // a4
    const double ca = tf.cosa();
    const double sa = tf.sina();
    const double cam1 = ca - 1;   // cam1
    const double nx2 = nx * nx;   // nx2
    const double ny2 = ny * ny;   // ny2
    const double nz2 = nz * nz;   // nz2
    const double nxny = nx * ny;  // nxny
    const double nynz = ny * nz;  // nynz
    const double nxnz = nx * nz;  // nxnz

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dzdx;
    half_d2F_dzdx(0, 0) =
            (((2 * ny *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * nz *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 -
                     (2 * nx * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            (((ny * (rxry + a * rz * sa - ca * rxry)) / a2 - (nz * (a * ry * sa - rxrz + ca * rxrz)) / a2 +
                     (nx * (a2 * ca - ca * rx2 + rx2)) / a2) *
                    (2 * nx *
                                    ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 -
                                            (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * ny *
                                    ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                            2 * nz *
                                    ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 +
                                            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2;
    half_d2F_dzdx(0, 1) =
            (((nz * (ryrz + a * rx * sa - ca * ryrz)) / a2 - (nx * (a * rz * sa - rxry + ca * rxry)) / a2 +
                     (ny * (a2 * ca - ca * ry2 + ry2)) / a2) *
                    (2 * nx *
                                    ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 -
                                            (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * ny *
                                    ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                            2 * nz *
                                    ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 +
                                            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2 +
            (((2 * nx *
                      (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                              a * rx2ry * sa)) /
                             a4 +
                     (2 * nz *
                             (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * ny * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(0, 2) =
            (((nx * (rxrz + a * ry * sa - ca * rxrz)) / a2 - (ny * (a * rx * sa - ryrz + ca * ryrz)) / a2 +
                     (nz * (a2 * ca - ca * rz2 + rz2)) / a2) *
                    (2 * nx *
                                    ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 -
                                            (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * ny *
                                    ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * rx2ry * sa)) /
                                                    a4 +
                                            (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                            2 * nz *
                                    ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                   a2 * ca * rxry + a * rx2rz * sa)) /
                                                    a4 +
                                            (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2 -
            (((2 * ny * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * nx *
                             (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * rx2rz * sa)) /
                             a4 +
                     (2 * nz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(0, 3) =
            -(nx * (2 * nx *
                                   ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 -
                                           (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * ny *
                                   ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                           2 * nz *
                                   ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                  a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 +
                                           (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(0, 4) =
            -(ny * (2 * nx *
                                   ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 -
                                           (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * ny *
                                   ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                           2 * nz *
                                   ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                  a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 +
                                           (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(0, 5) =
            -(nz * (2 * nx *
                                   ((py * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 -
                                           (px * rx * (a2 - rx2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * ny *
                                   ((pz * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (px * (a2 * ry - 2 * rx2ry + 2 * ca * rx2ry - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * rx2ry * sa)) /
                                                   a4 +
                                           (py * rx * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) +
                           2 * nz *
                                   ((px * (a2 * rz - 2 * rx2rz + 2 * ca * rx2rz - a2 * ca * rz + a * rxry * sa -
                                                  a2 * ca * rxry + a * rx2rz * sa)) /
                                                   a4 +
                                           (py * (a3 * sa - a * rx2 * sa + a2 * ca * rx2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * rx * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(1, 0) =
            (((ny * (rxry + a * rz * sa - ca * rxry)) / a2 - (nz * (a * ry * sa - rxrz + ca * rxrz)) / a2 +
                     (nx * (a2 * ca - ca * rx2 + rx2)) / a2) *
                    (2 * ny *
                                    ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 -
                                            (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                            2 * nx *
                                    ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                            2 * nz *
                                    ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 +
                                            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2 -
            (((2 * nz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * ny *
                             (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                                     a * rxry2 * sa)) /
                             a4 +
                     (2 * nx * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(1, 1) =
            (((2 * nx *
                      (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxry2 * sa)) /
                             a4 +
                     (2 * nz *
                             (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa + a2 * ca * rxry +
                                     a * ry2rz * sa)) /
                             a4 -
                     (2 * ny * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            (((nz * (ryrz + a * rx * sa - ca * ryrz)) / a2 - (nx * (a * rz * sa - rxry + ca * rxry)) / a2 +
                     (ny * (a2 * ca - ca * ry2 + ry2)) / a2) *
                    (2 * ny *
                                    ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 -
                                            (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                            2 * nx *
                                    ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                            2 * nz *
                                    ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 +
                                            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2;
    half_d2F_dzdx(1, 2) =
            (((nx * (rxrz + a * ry * sa - ca * rxrz)) / a2 - (ny * (a * rx * sa - ryrz + ca * ryrz)) / a2 +
                     (nz * (a2 * ca - ca * rz2 + rz2)) / a2) *
                    (2 * ny *
                                    ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 -
                                            (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                            2 * nx *
                                    ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxry2 * sa)) /
                                                    a4 +
                                            (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                            2 * nz *
                                    ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                          a2 * ca * rxry + a * ry2rz * sa)) /
                                                    a4 +
                                            (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
                    2 +
            (((2 * ny *
                      (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa - a2 * ca * rxry +
                              a * ry2rz * sa)) /
                             a4 +
                     (2 * nx *
                             (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * nz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(1, 3) =
            -(nx * (2 * ny *
                                   ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 -
                                           (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                           2 * nx *
                                   ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                           2 * nz *
                                   ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 +
                                           (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(1, 4) =
            -(ny * (2 * ny *
                                   ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 -
                                           (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                           2 * nx *
                                   ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                           2 * nz *
                                   ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 +
                                           (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(1, 5) =
            -(nz * (2 * ny *
                                   ((px * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx - a * ryrz * sa +
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz + a * rxry * sa -
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 -
                                           (py * ry * (a2 - ry2) * (2 * ca + a * sa - 2)) / a4) +
                           2 * nx *
                                   ((py * (a2 * rx - 2 * rxry2 + 2 * ca * rxry2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxry2 * sa)) /
                                                   a4 +
                                           (pz * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (px * ry * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) -
                           2 * nz *
                                   ((px * (a3 * sa - a * ry2 * sa + a2 * ca * ry2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (py * (a2 * rz - 2 * ry2rz + 2 * ca * ry2rz - a2 * ca * rz - a * rxry * sa +
                                                         a2 * ca * rxry + a * ry2rz * sa)) /
                                                   a4 +
                                           (pz * ry * (a3 * sa - 2 * ca * rz2 + 2 * rz2 - a * rz2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(2, 0) =
            (((ny * (rxry + a * rz * sa - ca * rxry)) / a2 - (nz * (a * ry * sa - rxrz + ca * rxrz)) / a2 +
                     (nx * (a2 * ca - ca * rx2 + rx2)) / a2) *
                    (2 * nz *
                                    ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 -
                                            (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * nx *
                                    ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                          a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                            2 * ny *
                                    ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 +
                                            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
                    2 +
            (((2 * nz *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa - a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * ny *
                             (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz + 2 * ca * rxryrz +
                                     a * rxryrz * sa)) /
                             a4 -
                     (2 * nx * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(2, 1) =
            (((nz * (ryrz + a * rx * sa - ca * ryrz)) / a2 - (nx * (a * rz * sa - rxry + ca * rxry)) / a2 +
                     (ny * (a2 * ca - ca * ry2 + ry2)) / a2) *
                    (2 * nz *
                                    ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 -
                                            (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * nx *
                                    ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                          a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                            2 * ny *
                                    ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 +
                                            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
                    2 -
            (((2 * nx * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz - a * rxryrz * sa)) /
                             a4 -
                     (2 * nz *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa + a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 +
                     (2 * ny * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2;
    half_d2F_dzdx(2, 2) =
            (((2 * nx *
                      (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa + a2 * ca * ryrz +
                              a * rxrz2 * sa)) /
                             a4 +
                     (2 * ny *
                             (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa - a2 * ca * rxrz +
                                     a * ryrz2 * sa)) /
                             a4 -
                     (2 * nz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) *
                    ((nx * (a2 * tx - a2 * qx + px * rx2 - ca * px * rx2 + py * rxry + pz * rxrz + a2 * ca * px -
                                   a * py * rz * sa + a * pz * ry * sa - ca * py * rxry - ca * pz * rxrz)) /
                                    a2 +
                            (ny * (a2 * ty - a2 * qy + py * ry2 - ca * py * ry2 + px * rxry + pz * ryrz + a2 * ca * py +
                                          a * px * rz * sa - a * pz * rx * sa - ca * px * rxry - ca * pz * ryrz)) /
                                    a2 +
                            (nz * (a2 * tz - a2 * qz + pz * rz2 - ca * pz * rz2 + px * rxrz + py * ryrz + a2 * ca * pz -
                                          a * px * ry * sa + a * py * rx * sa - ca * px * rxrz - ca * py * ryrz)) /
                                    a2)) /
                    2 +
            (((nx * (rxrz + a * ry * sa - ca * rxrz)) / a2 - (ny * (a * rx * sa - ryrz + ca * ryrz)) / a2 +
                     (nz * (a2 * ca - ca * rz2 + rz2)) / a2) *
                    (2 * nz *
                                    ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                   a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                          a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 -
                                            (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                            2 * nx *
                                    ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                   a * rxryrz * sa)) /
                                                    a4 -
                                            (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                          a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                    a4 +
                                            (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                            2 * ny *
                                    ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                   a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                    a4 +
                                            (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                          2 * ca * rxryrz + a * rxryrz * sa)) /
                                                    a4 -
                                            (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
                    2;
    half_d2F_dzdx(2, 3) =
            -(nx * (2 * nz *
                                   ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 -
                                           (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * nx *
                                   ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                         a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                           2 * ny *
                                   ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 +
                                           (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(2, 4) =
            -(ny * (2 * nz *
                                   ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 -
                                           (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * nx *
                                   ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                         a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                           2 * ny *
                                   ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 +
                                           (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(2, 5) =
            -(nz * (2 * nz *
                                   ((px * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx + a * ryrz * sa -
                                                  a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (py * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry - a * rxrz * sa +
                                                         a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 -
                                           (pz * rz * (a2 - rz2) * (2 * ca + a * sa - 2)) / a4) -
                           2 * nx *
                                   ((py * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 + 2 * rxryrz - 2 * ca * rxryrz -
                                                  a * rxryrz * sa)) /
                                                   a4 -
                                           (pz * (a2 * rx - 2 * rxrz2 + 2 * ca * rxrz2 - a2 * ca * rx - a * ryrz * sa +
                                                         a2 * ca * ryrz + a * rxrz2 * sa)) /
                                                   a4 +
                                           (px * rz * (a3 * sa - 2 * ca * rx2 + 2 * rx2 - a * rx2 * sa)) / a4) +
                           2 * ny *
                                   ((pz * (a2 * ry - 2 * ryrz2 + 2 * ca * ryrz2 - a2 * ca * ry + a * rxrz * sa -
                                                  a2 * ca * rxrz + a * ryrz2 * sa)) /
                                                   a4 +
                                           (px * (a3 * sa - a * rz2 * sa + a2 * ca * rz2 - 2 * rxryrz +
                                                         2 * ca * rxryrz + a * rxryrz * sa)) /
                                                   a4 -
                                           (py * rz * (a3 * sa - 2 * ca * ry2 + 2 * ry2 - a * ry2 * sa)) / a4))) /
            2;
    half_d2F_dzdx(3, 0) = nx * (nx * (ca - (rx2 * (cam1)) / a2) + ny * ((rz * sa) / a - (rxry * (cam1)) / a2) -
                                       nz * ((ry * sa) / a + (rxrz * (cam1)) / a2));
    half_d2F_dzdx(3, 1) = nx * (ny * (ca - (ry2 * (cam1)) / a2) - nx * ((rz * sa) / a + (rxry * (cam1)) / a2) +
                                       nz * ((rx * sa) / a - (ryrz * (cam1)) / a2));
    half_d2F_dzdx(3, 2) = nx * (nz * (ca - (rz2 * (cam1)) / a2) + nx * ((ry * sa) / a - (rxrz * (cam1)) / a2) -
                                       ny * ((rx * sa) / a + (ryrz * (cam1)) / a2));
    half_d2F_dzdx(3, 3) = -nx2;
    half_d2F_dzdx(3, 4) = -nx * ny;
    half_d2F_dzdx(3, 5) = -nx * nz;
    half_d2F_dzdx(4, 0) = ny * (nx * (ca - (rx2 * (cam1)) / a2) + ny * ((rz * sa) / a - (rxry * (cam1)) / a2) -
                                       nz * ((ry * sa) / a + (rxrz * (cam1)) / a2));
    half_d2F_dzdx(4, 1) = ny * (ny * (ca - (ry2 * (cam1)) / a2) - nx * ((rz * sa) / a + (rxry * (cam1)) / a2) +
                                       nz * ((rx * sa) / a - (ryrz * (cam1)) / a2));
    half_d2F_dzdx(4, 2) = ny * (nz * (ca - (rz2 * (cam1)) / a2) + nx * ((ry * sa) / a - (rxrz * (cam1)) / a2) -
                                       ny * ((rx * sa) / a + (ryrz * (cam1)) / a2));
    half_d2F_dzdx(4, 3) = -nx * ny;
    half_d2F_dzdx(4, 4) = -ny2;
    half_d2F_dzdx(4, 5) = -ny * nz;
    half_d2F_dzdx(5, 0) = nz * (nx * (ca - (rx2 * (cam1)) / a2) + ny * ((rz * sa) / a - (rxry * (cam1)) / a2) -
                                       nz * ((ry * sa) / a + (rxrz * (cam1)) / a2));
    half_d2F_dzdx(5, 1) = nz * (ny * (ca - (ry2 * (cam1)) / a2) - nx * ((rz * sa) / a + (rxry * (cam1)) / a2) +
                                       nz * ((rx * sa) / a - (ryrz * (cam1)) / a2));
    half_d2F_dzdx(5, 2) = nz * (nz * (ca - (rz2 * (cam1)) / a2) + nx * ((ry * sa) / a - (rxrz * (cam1)) / a2) -
                                       ny * ((rx * sa) / a + (ryrz * (cam1)) / a2));
    half_d2F_dzdx(5, 3) = -nx * nz;
    half_d2F_dzdx(5, 4) = -ny * nz;
    half_d2F_dzdx(5, 5) = -nz2;
    return half_d2F_dzdx;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpLinearisedModel<PointSource, PointTarget>::compute_half_d2F_dx2(
        const eigen_ext::PrecomputedTransformComponents<double>&, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) {
    // Compute required quantities once
    const Eigen::Vector3d p_cross_n = p.cross(n);
    const Eigen::Matrix3d p_cross_n_times_nT = p.cross(n) * n.transpose();

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dx2;
    half_d2F_dx2 << p_cross_n * p_cross_n.transpose(), p_cross_n_times_nT, p_cross_n_times_nT.transpose(),
            n * n.transpose();
    return half_d2F_dx2;
}

template<typename PointSource, typename PointTarget>
Eigen::Matrix<double, 6, 6> PointToPlaneIcpLinearisedModel<PointSource, PointTarget>::compute_half_d2F_dzdx(
        const eigen_ext::PrecomputedTransformComponents<double>& tf, const Eigen::Matrix<double, 3, 1>& p,
        const Eigen::Matrix<double, 3, 1>& q, const Eigen::Matrix<double, 3, 1>& n) {
    // Compute required quantities once
    const Eigen::Vector3d p_cross_n = p.cross(n);
    const Eigen::RowVector3d n_cross_r_plus_n_transpose = (n.cross(tf.r()) + n).transpose();

    // Fill elements
    Eigen::Matrix<double, 6, 6> half_d2F_dzdx;
    half_d2F_dzdx << (n.dot(q - p - tf.t()) - p_cross_n.dot(tf.r())) * eigen_ext::skew_symmetric(n) +
                             p_cross_n * n_cross_r_plus_n_transpose,
            -p_cross_n * n.transpose(), n * n_cross_r_plus_n_transpose, -n * n.transpose();
    return half_d2F_dzdx;
}

}
