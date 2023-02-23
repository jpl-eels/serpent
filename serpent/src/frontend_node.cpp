#include <ros/ros.h>

#include <pointcloud_tools/point_types.hpp>

#include "serpent/frontend.hpp"
#include "serpent/pointcloud_add_unit_vectors.hpp"
#include "serpent/pointcloud_filter.hpp"
#include "serpent/pointcloud_formatter.hpp"
#include "serpent/pointcloud_normal_estimation.hpp"

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent_frontend");
    ros::NodeHandle nh("serpent");

    // Initialise Modules
    serpent::Frontend frontend;
    serpent::PointcloudFormatter pointcloud_formatter;
    std::unique_ptr<serpent::PointcloudFilter> pointcloud_filter;
    std::unique_ptr<serpent::PointcloudAddUnitVectors> pointcloud_add_unit_vectors;
    std::unique_ptr<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>> normal_estimation;
    std::unique_ptr<serpent::PointcloudNormalEstimation<PointUnit, PointNormalUnit>> normal_estimation_unit_vectors;
    if (nh.param<bool>("optimisation/factors/registration", true)) {
        const bool voxel_grid_filter_enabled = nh.param<bool>("voxel_grid_filter/enabled", true);
        const bool add_unit_vectors =
                serpent::requires_unit_vectors(nh.param<std::string>("registration_covariance/method", "CENSI"),
                        nh.param<std::string>("registration_covariance/point_covariance/method", "RANGE"));
        ROS_WARN_COND(voxel_grid_filter_enabled && add_unit_vectors,
                "Both the VoxelGrid filter and unit vector addition are enabled, however the unit vectors will not "
                "exactly match the sensor models used for covariance estimation.");
        const bool filter_enabled = voxel_grid_filter_enabled || nh.param<bool>("body_filter/enabled", false) ||
                                    nh.param<bool>("range_filter/enabled", true) ||
                                    nh.param<bool>("statistical_outlier_removal/enabled", false) ||
                                    nh.param<bool>("random_sample_filter/enabled", false);
        std::string input_topic = frontend.output_pointcloud_topic();
        if (filter_enabled) {
            pointcloud_filter = std::make_unique<serpent::PointcloudFilter>(input_topic);
            input_topic = pointcloud_filter->output_topic();
        }
        if (add_unit_vectors) {
            pointcloud_add_unit_vectors = std::make_unique<serpent::PointcloudAddUnitVectors>(input_topic);
            input_topic = pointcloud_add_unit_vectors->output_topic();
            ROS_INFO_STREAM("Add unit vectors enabled. Building modules with PointUnit and PointNormalUnit types.");
            normal_estimation_unit_vectors =
                    std::make_unique<serpent::PointcloudNormalEstimation<PointUnit, PointNormalUnit>>(input_topic);
        } else {
            ROS_INFO_STREAM("Add unit vectors not enabled. Building modules with pcl::PointXYZ and pcl::PointNormal "
                            " types.");
            normal_estimation =
                    std::make_unique<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>>(input_topic);
        }
    }

    // Start the Node
    const int threads = nh.param<int>("threads", 4);
    if (threads < 1) {
        throw std::runtime_error("Thread count must be >= 1, was " + std::to_string(threads));
    }
    ros::AsyncSpinner spinner(threads);
    spinner.start();
    ROS_INFO_STREAM("Spinning with " << std::to_string(threads) << " threads");
    ros::waitForShutdown();
    std::cerr << "Shutdown complete. Exitting." << std::endl;
    return 0;
}
