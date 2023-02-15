#ifndef SERPENT_REGISTRATION_METHODS_HPP
#define SERPENT_REGISTRATION_METHODS_HPP

#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <ros/ros.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/lsq_registration.hpp>

namespace fast_gicp {

RegularizationMethod to_regularization_method(const std::string& method);

VoxelAccumulationMode to_voxel_accumulation_mode(const std::string& mode);

NeighborSearchMethod to_neighbor_search_method(const std::string& method);

}

namespace serpent {

template<typename PointIn, typename PointOut>
void set_registration_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::Registration<PointIn, PointOut>& registration);

template<typename PointIn, typename PointOut>
void set_icp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::IterativeClosestPoint<PointIn, PointOut>& icp);

template<typename PointIn, typename PointOut>
void set_gicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::GeneralizedIterativeClosestPoint<PointIn, PointOut>& gicp);

template<typename PointIn, typename PointOut>
void set_lsq_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::LsqRegistration<PointIn, PointOut>& lsq) ;

template<typename PointIn, typename PointOut>
void set_fast_gicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastGICP<PointIn, PointOut>& fast_gicp);

template<typename PointIn, typename PointOut>
void set_fast_gicp_st_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastGICPSingleThread<PointIn, PointOut>& fast_gicp_st);

template<typename PointIn, typename PointOut>
void set_fast_vgicp_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename fast_gicp::FastVGICP<PointIn, PointOut>& fast_vgicp);

template<typename PointIn, typename PointOut>
void set_ndt_parameters(const ros::NodeHandle& nh, const std::string& prefix,
        typename pcl::NormalDistributionsTransform<PointIn, PointOut>& ndt);

template<typename PointIn, typename PointOut>
typename pcl::Registration<PointIn, PointOut>::Ptr registration_method(const ros::NodeHandle& nh,
        const std::string& prefix);

}

#include "serpent/impl/registration_methods.hpp"

#endif
