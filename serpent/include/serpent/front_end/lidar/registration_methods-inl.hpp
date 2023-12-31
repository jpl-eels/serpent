#ifndef SERPENT_IMPL_REGISTRATION_METHODS_HPP
#define SERPENT_IMPL_REGISTRATION_METHODS_HPP

#include "serpent/front_end/lidar/registration_methods.hpp"

namespace serpent {

template<typename PointIn, typename PointOut>
void set_registration_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::Registration<PointIn, PointOut>& registration) {
    registration.setMaximumIterations(nh.param<int>(prefix + "base/maximum_iterations", 10));
    registration.setRANSACIterations(nh.param<int>(prefix + "base/ransac_iterations", 0));
    registration.setRANSACOutlierRejectionThreshold(
            nh.param<double>(prefix + "base/ransac_outlier_rejection_threshold", 0.05));
    registration.setMaxCorrespondenceDistance(
            nh.param<double>(prefix + "base/max_correspondence_distance", std::numeric_limits<double>::max()));
    registration.setTransformationEpsilon(nh.param<double>(prefix + "base/transformation_epsilon", 0.0));
    registration.setTransformationRotationEpsilon(
            nh.param<double>(prefix + "base/transformation_rotation_epsilon", 0.0));
    registration.setEuclideanFitnessEpsilon(nh.param<double>(prefix + "base/euclidean_fitness_epsilon", 0.0));
}

template<typename PointIn, typename PointOut>
void set_icp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::IterativeClosestPoint<PointIn, PointOut>& icp) {
    set_registration_parameters(nh, prefix, icp);
    icp.setUseReciprocalCorrespondences(nh.param<bool>(prefix + "icp/use_reciprocal_correspondences", false));
}

template<typename PointIn, typename PointOut>
void set_gicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::GeneralizedIterativeClosestPoint<PointIn, PointOut>& gicp) {
    set_icp_parameters(nh, prefix, gicp);
    gicp.setRotationEpsilon(nh.param<double>(prefix + "gicp/rotation_epsilon", 2.0e-3));
    gicp.setCorrespondenceRandomness(nh.param<int>(prefix + "gicp/correspondence_randomness", 20));
    gicp.setMaximumOptimizerIterations(nh.param<int>(prefix + "gicp/maximum_optimizer_iterations", 20));
    // From PCL 1.11.0:
    // gicp.setTranslationGradientTolerance(nh.param<double>(prefix + "gicp/translation_gradient_tolerance", 1.0e-2));
    // gicp.setRotationGradientTolerance(nh.param<double>(prefix + "gicp/rotation_gradient_tolerance", 1.0e-2));
}

template<typename PointIn, typename PointOut>
void set_lsq_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::LsqRegistration<PointIn, PointOut>& lsq) {
    set_registration_parameters(nh, prefix, lsq);
    lsq.setRotationEpsilon(nh.param<double>(prefix + "lsq/rotation_epsilon", 2.0e-3));
    lsq.setInitialLambdaFactor(nh.param<double>(prefix + "lsq/initial_lambda_factor", 1.0e-9));
    lsq.setDebugPrint(nh.param<bool>(prefix + "lsq/debug_print", false));
}

template<typename PointIn, typename PointOut>
void set_fast_gicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastGICP<PointIn, PointOut>& fast_gicp) {
    set_lsq_parameters(nh, prefix, fast_gicp);
    fast_gicp.setNumThreads(nh.param<int>(prefix + "fast_gicp/num_threads", 4));
    fast_gicp.setCorrespondenceRandomness(nh.param<int>(prefix + "fast_gicp/correspondence_randomness", 20));
    fast_gicp.setRegularizationMethod(fast_gicp::to_regularization_method(
            nh.param<std::string>(prefix + "fast_gicp/regularization_method", "PLANE")));
}

template<typename PointIn, typename PointOut>
void set_fast_gicp_st_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastGICPSingleThread<PointIn, PointOut>& fast_gicp_st) {
    set_fast_gicp_parameters(nh, prefix, fast_gicp_st);
    fast_gicp_st.setNumThreads(1);
}

template<typename PointIn, typename PointOut>
void set_fast_vgicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastVGICP<PointIn, PointOut>& fast_vgicp) {
    set_fast_gicp_parameters(nh, prefix, fast_vgicp);
    fast_vgicp.setResolution(nh.param<double>(prefix + "fast_vgicp/resolution", 1.0));
    fast_vgicp.setVoxelAccumulationMode(fast_gicp::to_voxel_accumulation_mode(
            nh.param<std::string>(prefix + "fast_vgicp/voxel_accumulation_mode", "ADDITIVE")));
    fast_vgicp.setNeighborSearchMethod(fast_gicp::to_neighbor_search_method(
            nh.param<std::string>(prefix + "fast_vgicp/neighbor_search_method", "DIRECT1")));
}

template<typename PointIn, typename PointOut>
void set_ndt_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::NormalDistributionsTransform<PointIn, PointOut>& ndt) {
    set_registration_parameters(nh, prefix, ndt);
    // overwrite max iterations default value to 35 for ndt if base/maximum_iterations is not set
    ndt.setMaximumIterations(nh.param<int>(prefix + "base/maximum_iterations", 35));
    ndt.setResolution(nh.param<float>(prefix + "ndt/resolution", 1.0f));
    ndt.setStepSize(nh.param<double>(prefix + "ndt/step_size", 0.1));
    ndt.setOulierRatio(nh.param<double>(prefix + "ndt/outlier_ratio", 0.55));  // Note pcl function typo
}

template<typename PointIn, typename PointOut>
typename pcl::Registration<PointIn, PointOut>::Ptr registration_method(
        const ros::NodeHandle& nh, const std::string& prefix) {
    const std::string method = nh.param<std::string>(prefix + "method", "fast_gicp");
    typename pcl::Registration<PointIn, PointOut>::Ptr registration;
    if (method == "icp") {
        auto icp = boost::make_shared<typename pcl::IterativeClosestPoint<PointIn, PointOut>>();
        set_icp_parameters(nh, prefix, *icp);
        registration = icp;
    } else if (method == "gicp") {
        auto gicp = boost::make_shared<typename pcl::GeneralizedIterativeClosestPoint<PointIn, PointOut>>();
        set_gicp_parameters(nh, prefix, *gicp);
        registration = gicp;
    } else if (method == "fast_gicp") {
        auto fast_gicp = boost::make_shared<typename fast_gicp::FastGICP<PointIn, PointOut>>();
        set_fast_gicp_parameters(nh, prefix, *fast_gicp);
        registration = fast_gicp;
    } else if (method == "fast_gicp_st") {
        auto fast_gicp_st = boost::make_shared<typename fast_gicp::FastGICPSingleThread<PointIn, PointOut>>();
        set_fast_gicp_st_parameters(nh, prefix, *fast_gicp_st);
        registration = fast_gicp_st;
    } else if (method == "fast_vgicp") {
        auto fast_vgicp = boost::make_shared<typename fast_gicp::FastVGICP<PointIn, PointOut>>();
        set_fast_vgicp_parameters(nh, prefix, *fast_vgicp);
        registration = fast_vgicp;
    } else if (method == "ndt") {
        auto ndt = boost::make_shared<typename pcl::NormalDistributionsTransform<PointIn, PointOut>>();
        set_ndt_parameters(nh, prefix, *ndt);
        registration = ndt;
    } else {
        throw std::runtime_error("Registration method \'" + method + "\' not supported");
    }
    return registration;
}

}

#endif
