#ifndef CAMERA_TOOLS_IMAGE_OPERATIONS_HPP
#define CAMERA_TOOLS_IMAGE_OPERATIONS_HPP

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

namespace camera_tools {

/**
 * @brief Base class for image operations.
 *
 */
class ImageOperation {
public:
    virtual cv::Mat apply(const cv::Mat& mat) = 0;

    virtual sensor_msgs::CameraInfo apply(const sensor_msgs::CameraInfo& info) = 0;
};

/**
 * @brief Rotation image operation. Defines rotations as multiples of 90 degrees.
 *
 */
class Rotate : public ImageOperation {
public:
    Rotate(const cv::RotateFlags option);

    Rotate(int degrees_clockwise);

    cv::Mat apply(const cv::Mat& in) override;

    sensor_msgs::CameraInfo apply(const sensor_msgs::CameraInfo& info) override;

private:
    cv::RotateFlags option;
};

/**
 * @brief Flip image operation. Capable of flipping around the horizontal axis, vertical axis, or both.
 *
 */
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

}

#endif
