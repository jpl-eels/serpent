#include "imu_tools/imu_analysis.hpp"
#include <eigen_ros/eigen_ros.hpp>
#include <eigen_ros/geometry_msgs.hpp>
#include <eigen_ros/sensor_msgs.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <sstream>

ImuAnalysis::ImuAnalysis()
    :   nh("~") {
    imu_subscriber = nh.subscribe<sensor_msgs::Imu>(nh.param<std::string>("imu_topic", "imu"), 1000,
            &ImuAnalysis::analyse, this);
    fixed_frame = nh.param<std::string>("fixed_frame", "fixed");
    apply_transform = nh.param<bool>("apply_transform", false);
    transformed_frame = nh.param<std::string>("transformed_frame", "fixed");

    std::vector<double> translation_vec = nh.param<std::vector<double>>("translation", {{0, 0, 0}});
    for (std::size_t i = 0; i < 3; ++i) {
        translation(i) = translation_vec.at(i);
    }
    std::vector<double> rotation_vec = nh.param<std::vector<double>>("rotation", {{1, 0, 0, 0, 1, 0, 0, 0, 1}});
    Eigen::Matrix3d rotation_mat;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            rotation_mat(r, c) = rotation_vec.at(r*3 + c);
        }
    }
    rotation = Eigen::Quaterniond(rotation_mat);
    if (apply_transform) {
        std::cerr << "An additional TF will be displayed with the following translation:\n"
                << "[" << translation(0) << ", " << translation(1) << ", " << translation(2) << "]\n"
                << "and rotation:\n"
                << rotation_mat.matrix() << "\n";
    }
}

void ImuAnalysis::analyse(const sensor_msgs::ImuConstPtr& msg) {
    // Publish TF
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = fixed_frame;
    tf.child_frame_id = msg->header.frame_id;
    tf.transform.rotation = msg->orientation;
    tf_broadcaster.sendTransform(tf);

    // Convert to Eigen
    eigen_ros::Imu imu = eigen_ros::from_ros<eigen_ros::Imu>(*msg);
    imu.print();

    if (apply_transform) {
        // Note we apply rotation then q since R_FX = R_FI * R_IX = q_FI * q_IX, where F is the fixed frame, X is the 
        // new frame and I is the IMU frame.
        tf.child_frame_id = transformed_frame;
        tf.transform.translation.x = translation(0);
        tf.transform.translation.y = translation(1);
        tf.transform.translation.z = translation(2);
        const Eigen::Quaterniond q_FX = imu.orientation * rotation;
        eigen_ros::to_ros(tf.transform.rotation, q_FX);
        tf_broadcaster.sendTransform(tf);

        const Eigen::Vector3d rpy_FX = eigen_ros::rpy(q_FX);
        std::stringstream ss;
        ss << std::setprecision(3);
        ss  << "Applied TF (" << tf.header.frame_id << " => " << tf.child_frame_id << ")\n"
            << "\ttranslation: [" << translation(0) << ", " << translation(1) << ", " << translation(2) << "]\n"
            << "\torientation:\n"
            << "\t\tquaternion (wxyz): [" << q_FX.w() << ", " << q_FX.x() << ", " << q_FX.y() << ", " << q_FX.z() << "]\n"
            << "\t\tRPY (radians):     [" << rpy_FX[0] << ", " << rpy_FX[1] << ", " << rpy_FX[2] << "]\n"
            << "\t\tRPY (degrees):     [" << 180.0 * rpy_FX[0] / M_PI << ", " << 180.0 * rpy_FX[1] / M_PI << ", "
            << 180.0 * rpy_FX[2] / M_PI << "]\n";
        std::cerr << ss.str();
    }
}
