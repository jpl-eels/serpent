#include "camera_tools/publish_info.hpp"

namespace camera_tools {

PublishInfo::PublishInfo()
    : nh("~") {
    // Subscribers
    image_subscriber = nh.subscribe<sensor_msgs::Image>("input/image", 10, &PublishInfo::callback, this);

    // Publishers
    camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("output/camera_info", 1);

    // Create camera info from config
    nh.param<std::string>("distortion_model", camera_info.distortion_model, "");
    nh.param<std::vector<double>>("distortion_coefficients", camera_info.D, std::vector<double>{});
    camera_info.K[0] = nh.param<double>("K/fx", 0.0);
    camera_info.K[2] = nh.param<double>("K/cx", 0.0);
    camera_info.K[4] = nh.param<double>("K/fy", 0.0);
    camera_info.K[5] = nh.param<double>("K/cy", 0.0);
    camera_info.K[8] = 1.0;

    camera_info.R[0] = 1.0;
    camera_info.R[4] = 1.0;
    camera_info.R[8] = 1.0;

    camera_info.P[0] = nh.param<double>("P/fx", 0.0);
    camera_info.P[2] = nh.param<double>("P/cx", 0.0);
    camera_info.P[3] = - camera_info.P[0] * nh.param<double>("P/baseline", 0.0);
    camera_info.P[5] = nh.param<double>("P/fy", 0.0);
    camera_info.P[6] = nh.param<double>("P/cy", 0.0);
    camera_info.P[10] = 1.0;
}

void PublishInfo::callback(const sensor_msgs::ImageConstPtr& msg) {
    camera_info.header = msg->header;
    camera_info.height = msg->height;
    camera_info.width = msg->width;
    camera_info_publisher.publish(camera_info);
}

}
