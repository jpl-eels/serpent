#ifndef CAMERA_TOOLS_SPLIT_IMAGE_HPP
#define CAMERA_TOOLS_SPLIT_IMAGE_HPP

#include <image_transport/image_transport.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <memory>
#include <optional>

namespace camera_tools {

class ImageOperation {
public:
    virtual cv::Mat apply(const cv::Mat& mat) = 0;

    virtual sensor_msgs::CameraInfo apply(const sensor_msgs::CameraInfo& info) = 0;
};

class Rotate : public ImageOperation {
public:
    Rotate(const cv::RotateFlags option);

    Rotate(int degrees_clockwise);

    cv::Mat apply(const cv::Mat& in) override;

    sensor_msgs::CameraInfo apply(const sensor_msgs::CameraInfo& info) override;

private:
    cv::RotateFlags option;
};

class Flip : public ImageOperation {
public:
    enum Option {
        FLIP_HORIZ,
        FLIP_VERT,
        FLIP_BOTH
    };

    Flip(const Option option);

    cv::Mat apply(const cv::Mat& in) override;

    sensor_msgs::CameraInfo apply(const sensor_msgs::CameraInfo& info) override;

private:
    int flip_code;
};

struct ImageSplitConfig {
    image_transport::Publisher publisher;
    ros::Publisher info_publisher;
    std::string frame_id;
    cv::Rect region;
    sensor_msgs::CameraInfo camera_info;
    std::vector<std::shared_ptr<ImageOperation>> operations;
};

class ImageSplitter {
public:
    explicit ImageSplitter();

    void split(const sensor_msgs::ImageConstPtr& msg);

private:
    void parse_configuration();

    //// ROS Communications
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_subscriber;

    // Image regions
    std::vector<ImageSplitConfig> image_configs;
};

}

#endif
