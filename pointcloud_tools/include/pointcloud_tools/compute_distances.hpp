#ifndef POINTCLOUD_TOOLS_COMPUTE_DISTANCES_HPP
#define POINTCLOUD_TOOLS_COMPUTE_DISTANCES_HPP

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace pct {

template<typename PointSource, typename PointTarget>
class ComputeDistances {
public:
    using PointCloudSource = typename pcl::PointCloud<PointSource>;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
    using PointCloudTarget = typename pcl::PointCloud<PointTarget>;
    using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
    using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

    explicit ComputeDistances();

protected:
    void callback(const PointCloudSourceConstPtr& source, const PointCloudTargetConstPtr& target);

    virtual pcl::PCLPointCloud2::ConstPtr compute_distances(const PointCloudSource& source,
            const PointCloudTargetConstPtr transformed_target) const = 0;

    virtual PointCloudTargetConstPtr transform_target(const PointCloudTarget& target,
            const Eigen::Isometry3d& transform, const std::string& frame_id) const = 0;

private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    ros::Publisher output_publisher;
    message_filters::Subscriber<PointCloudSource> source_subscriber;
    message_filters::Subscriber<PointCloudTarget> target_subscriber;
    message_filters::TimeSynchronizer<PointCloudSource, PointCloudTarget> sync;
};

template<typename PointSource, typename PointTarget>
class ComputePointToPointDistances : public ComputeDistances<PointSource, PointTarget> {
    static_assert(std::is_floating_point<decltype(PointSource::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointSource::z)>::value, "z is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::x)>::value, "x is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::y)>::value, "y is not a floating point type");
    static_assert(std::is_floating_point<decltype(PointTarget::z)>::value, "z is not a floating point type");

public:
    using Base = ComputeDistances<PointSource, PointTarget>;
    using PointCloudSource = typename Base::PointCloudSource;
    using PointCloudSourcePtr = typename Base::PointCloudSourcePtr;
    using PointCloudSourceConstPtr = typename Base::PointCloudSourceConstPtr;
    using PointCloudTarget = typename Base::PointCloudTarget;
    using PointCloudTargetPtr = typename Base::PointCloudTargetPtr;
    using PointCloudTargetConstPtr = typename Base::PointCloudTargetConstPtr;

    explicit ComputePointToPointDistances() = default;

protected:
    pcl::PCLPointCloud2::ConstPtr compute_distances(const PointCloudSource& source,
            const PointCloudTargetConstPtr transformed_target) const override;

    PointCloudTargetConstPtr transform_target(const PointCloudTarget& target, const Eigen::Isometry3d& transform,
            const std::string& frame_id) const override;
};

}

#include "pointcloud_tools/impl/compute_distances.hpp"

#endif
