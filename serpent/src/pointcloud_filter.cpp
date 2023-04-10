#include "serpent/pointcloud_filter.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pointcloud_tools/pclpointcloud2_utilities.hpp>

namespace serpent {

PointcloudFilter::PointcloudFilter(std::string input_topic)
    : nh("serpent"),
     final_output_topic("filter/filtered_pointcloud") {
    const bool voxel_grid_enabled = nh.param<bool>("voxel_grid_filter/enabled", true);
    const bool body_filter_enabled = nh.param<bool>("body_filter/enabled", false);
    const bool range_filter_enabled = nh.param<bool>("range_filter/enabled", true);
    const bool sor_enabled = nh.param<bool>("statistical_outlier_removal/enabled", false);
    const bool random_sample_enabled = nh.param<bool>("random_sample_filter/enabled", false);
    ROS_INFO_STREAM("Filtering operations (in order):"
                    << "\nVOXEL_GRID    " << (voxel_grid_enabled ? " ENABLED" : "DISABLED") << "\nBODY_FILTER   "
                    << (body_filter_enabled ? " ENABLED" : "DISABLED") << "\nRANGE_FILTER  "
                    << (range_filter_enabled ? " ENABLED" : "DISABLED") << "\nSOR REMOVAL   "
                    << (sor_enabled ? " ENABLED" : "DISABLED") << "\nRANDOM SAMPLE "
                    << (random_sample_enabled ? " ENABLED" : "DISABLED"));

    // Voxel grid
    voxel_grid_filter.setDownsampleAllData(true);  // Necessary to keep all fields
    std::string output_topic = (random_sample_enabled || sor_enabled || range_filter_enabled || body_filter_enabled)
                                       ? "filter/voxel_filtered_pointcloud"
                                       : final_output_topic;
    if (voxel_grid_enabled) {
        const double leaf_size = nh.param<double>("voxel_grid_filter/leaf_size", 0.1);
        voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid_filter.setMinimumPointsNumberPerVoxel(
                nh.param<double>("voxel_grid_filter/minimum_points_number_per_voxel", 1));
        voxel_grid_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>(output_topic, 1);
        voxel_grid_pointcloud_subscriber =
                nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, &PointcloudFilter::voxel_grid_callback, this);
        input_topic = voxel_grid_pointcloud_publisher.getTopic();
    }

    // Body filter
    output_topic = (random_sample_enabled || sor_enabled || range_filter_enabled) ? "filter/body_filtered_pointcloud"
                                                                                  : final_output_topic;
    if (body_filter_enabled) {
        body_filter.setMin(Eigen::Vector4f(nh.param<float>("body_filter/min/x", 0.f),
                nh.param<float>("body_filter/min/y", 0.f), nh.param<float>("body_filter/min/z", 0.f), 1.f));
        body_filter.setMax(Eigen::Vector4f(nh.param<float>("body_filter/max/x", 0.f),
                nh.param<float>("body_filter/max/y", 0.f), nh.param<float>("body_filter/max/z", 0.f), 1.f));
        body_filter.setTranslation(Eigen::Vector3f(nh.param<float>("body_filter/translation/x", 0.f),
                nh.param<float>("body_filter/translation/y", 0.f), nh.param<float>("body_filter/translation/z", 0.f)));
        body_filter.setRotation(Eigen::Vector3f(nh.param<float>("body_filter/rotation/rx", 0.f),
                nh.param<float>("body_filter/rotation/ry", 0.f), nh.param<float>("body_filter/rotation/rz", 0.f)));
        body_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>(output_topic, 1);
        body_pointcloud_subscriber =
                nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, &PointcloudFilter::body_filter_callback, this);
        input_topic = body_pointcloud_publisher.getTopic();
    }

    // Range filter
    output_topic = (random_sample_enabled || sor_enabled) ? "filter/range_filtered_pointcloud" : final_output_topic;
    if (range_filter_enabled) {
        range_filter.setFilterFieldName("range");
        range_filter.setFilterLimits(nh.param<double>("range_filter/min", 0.0),
                nh.param<double>("range_filter/max", std::numeric_limits<double>::max()));
        range_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>(output_topic, 1);
        range_pointcloud_subscriber =
                nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, &PointcloudFilter::range_filter_callback, this);
        input_topic = range_pointcloud_publisher.getTopic();
    }

    // Statistical Outlier Removal filter
    output_topic = random_sample_enabled ? "filter/sor_filtered_pointcloud" : final_output_topic;
    if (sor_enabled) {
        sor_filter.setMeanK(nh.param<int>("statistical_outlier_removal_filter/mean_k", 1));
        sor_filter.setStddevMulThresh(nh.param<double>("statistical_outlier_removal_filter/stddev_mul_thresh", 0.0));
        statistical_outlier_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>(output_topic, 1);
        statistical_outlier_pointcloud_subscriber = nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100,
                &PointcloudFilter::statistical_outlier_callback, this);
        input_topic = statistical_outlier_pointcloud_publisher.getTopic();
    }

    // Random sample filter
    output_topic = final_output_topic;
    if (random_sample_enabled) {
        const int sample_size = nh.param<int>("random_sample_filter/sample_size", 3000);
        if (sample_size < 0) {
            throw std::runtime_error("RandomSample sample_size must be >= 0.");
        }
        const int seed = nh.param<int>("random_sample_filter/random_seed", 0);
        if (seed < 0) {
            throw std::runtime_error("RandomSample random_seed must be >= 0.");
        }
        random_sample_filter.setSample(static_cast<unsigned int>(sample_size));
        random_sample_filter.setSeed(static_cast<unsigned int>(seed));
        random_sample_pointcloud_publisher = nh.advertise<pcl::PCLPointCloud2>(output_topic, 1);
        random_sample_pointcloud_subscriber =
                nh.subscribe<pcl::PCLPointCloud2>(input_topic, 100, &PointcloudFilter::random_sample_callback, this);
        input_topic = random_sample_pointcloud_publisher.getTopic();
    }
}

std::string PointcloudFilter::output_topic() const {
    return final_output_topic;
}

pcl::PCLPointCloud2::Ptr filter(const pcl::PCLPointCloud2::ConstPtr& msg, pcl::Filter<pcl::PCLPointCloud2>& filter) {
    filter.setInputCloud(msg);
    auto pointcloud = boost::make_shared<pcl::PCLPointCloud2>();
    filter.filter(*pointcloud);
    return pointcloud;
}

void PointcloudFilter::body_filter_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    body_pointcloud_publisher.publish(filter(msg, body_filter));
}

void PointcloudFilter::random_sample_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    random_sample_pointcloud_publisher.publish(filter(msg, random_sample_filter));
}

void PointcloudFilter::range_filter_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    if (!pct::has_field(*msg, "range")) {
        throw std::runtime_error("Point cloud has no field \'range\' but range filter was enabled. Disable range filter"
                                 " or add \'range\' field to input point cloud.");
    }
    range_pointcloud_publisher.publish(filter(msg, range_filter));
}

void PointcloudFilter::statistical_outlier_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    statistical_outlier_pointcloud_publisher.publish(filter(msg, sor_filter));
}

void PointcloudFilter::voxel_grid_callback(const pcl::PCLPointCloud2::ConstPtr& msg) {
    voxel_grid_pointcloud_publisher.publish(filter(msg, voxel_grid_filter));
}

}
