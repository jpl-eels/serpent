#include <ros/ros.h>

#include <eigen_ros/body_frames_tf.hpp>
#include <memory>

#include "serpent/frontend.hpp"
#include "serpent/mapping.hpp"
#include "serpent/optimisation.hpp"
#include "serpent/pointcloud_add_unit_vectors.hpp"
#include "serpent/pointcloud_filter.hpp"
#include "serpent/pointcloud_formatter.hpp"
#include "serpent/pointcloud_normal_estimation.hpp"
#include "serpent/registration.hpp"
#include "serpent/stereo_factor_finder.hpp"

int main(int argc, char** argv) {
    // Initialise ROS
    ros::init(argc, argv, "serpent");
    ros::NodeHandle nh("serpent");

    // Initialise Modules
    eigen_ros::BodyFramesTf body_frames_tf("serpent");
    serpent::Frontend frontend;
    serpent::Optimisation optimisation;
    serpent::PointcloudFormatter pointcloud_formatter;
    std::unique_ptr<serpent::PointcloudFilter> pointcloud_filter;
    std::unique_ptr<serpent::PointcloudAddUnitVectors> pointcloud_add_unit_vectors;
    std::unique_ptr<serpent::Mapping<pcl::PointNormal>> mapping;
    std::unique_ptr<serpent::Mapping<PointNormalUnit>> mapping_unit_vectors;
    std::unique_ptr<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>> normal_estimation;
    std::unique_ptr<serpent::PointcloudNormalEstimation<PointUnit, PointNormalUnit>> normal_estimation_unit_vectors;
    std::unique_ptr<serpent::Registration<pcl::PointNormal>> registration;
    std::unique_ptr<serpent::Registration<PointNormalUnit>> registration_unit_vectors;
    std::unique_ptr<serpent::StereoFactorFinder> stereo_factor_finder;
    if (nh.param<bool>("optimisation/factors/registration", true)) {
        const bool voxel_grid_filter_enabled = nh.param<bool>("voxel_grid_filter/enabled", false);
        const bool add_unit_vectors =
                serpent::requires_unit_vectors(nh.param<std::string>("registration_covariance/method", "CENSI"),
                        nh.param<std::string>("registration_covariance/point_covariance/method", "RANGE"));
        if (voxel_grid_filter_enabled && add_unit_vectors) {
            ROS_ERROR("Both the VoxelGrid filter and unit vector addition are enabled, however the unit vectors will "
                      "be invalid for the sensor models used for covariance estimation. Disable the VoxelGrid filter "
                      "(use random sampling instead) or change registration covariance/point_covariance method.");
        }
        const bool filter_enabled = voxel_grid_filter_enabled || nh.param<bool>("body_filter/enabled", false) ||
                                    nh.param<bool>("range_filter/enabled", true) ||
                                    nh.param<bool>("statistical_outlier_removal/enabled", false) ||
                                    nh.param<bool>("random_sample_filter/enabled", true);
        std::string input_topic = frontend.output_pointcloud_topic();
        if (filter_enabled) {
            pointcloud_filter = std::make_unique<serpent::PointcloudFilter>(input_topic);
            input_topic = pointcloud_filter->output_topic();
        }
        if (add_unit_vectors) {
            pointcloud_add_unit_vectors = std::make_unique<serpent::PointcloudAddUnitVectors>(input_topic);
            input_topic = pointcloud_add_unit_vectors->output_topic();
            ROS_INFO_STREAM("Add unit vectors enabled. Building modules with PointUnit and PointNormalUnit "
                            "point types.");
            normal_estimation_unit_vectors =
                    std::make_unique<serpent::PointcloudNormalEstimation<PointUnit, PointNormalUnit>>(input_topic);
            mapping_unit_vectors = std::make_unique<serpent::Mapping<PointNormalUnit>>();
            registration_unit_vectors = std::make_unique<serpent::Registration<PointNormalUnit>>();
        } else {
            ROS_INFO_STREAM("Add unit vectors not enabled. Building modules with pcl::PointXYZ and pcl::PointNormal "
                            "point types.");
            normal_estimation =
                    std::make_unique<serpent::PointcloudNormalEstimation<pcl::PointXYZ, pcl::PointNormal>>(input_topic);
            mapping = std::make_unique<serpent::Mapping<pcl::PointNormal>>();
            registration = std::make_unique<serpent::Registration<pcl::PointNormal>>();
        }
    }
    if (nh.param<bool>("optimisation/factors/stereo", true)) {
        stereo_factor_finder = std::make_unique<serpent::StereoFactorFinder>();
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
