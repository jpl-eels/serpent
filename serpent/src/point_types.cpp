#include "serpent/point_types.hpp"

template class fast_gicp::FastGICPSingleThread<PointNormalUnit, PointNormalUnit>;
template class fast_gicp::FastGICP<PointNormalUnit, PointNormalUnit>;
template class fast_gicp::FastVGICP<PointNormalUnit, PointNormalUnit>;
template class fast_gicp::LsqRegistration<PointNormalUnit, PointNormalUnit>;
template class fast_gicp::FastGICPSingleThread<PointNormalCovariance, PointNormalCovariance>;
template class fast_gicp::FastGICP<PointNormalCovariance, PointNormalCovariance>;
template class fast_gicp::FastVGICP<PointNormalCovariance, PointNormalCovariance>;
template class fast_gicp::LsqRegistration<PointNormalCovariance, PointNormalCovariance>;
