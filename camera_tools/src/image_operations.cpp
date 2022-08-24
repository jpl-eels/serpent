#include "camera_tools/image_operations.hpp"

#include <ros/ros.h>

namespace camera_tools {

Rotate::Rotate(const cv::RotateFlags option)
    : option(option) {}

Rotate::Rotate(int degrees_clockwise) {
    degrees_clockwise = degrees_clockwise % 360;
    if (degrees_clockwise <= 0) {
        throw std::runtime_error("Rotate degrees_clockwise must be > 0");
    }
    if (degrees_clockwise == 90) {
        option = cv::RotateFlags::ROTATE_90_CLOCKWISE;
    } else if (degrees_clockwise == 180) {
        option = cv::RotateFlags::ROTATE_180;
    } else if (degrees_clockwise == 270) {
        option = cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;
    }
}

cv::Mat Rotate::apply(const cv::Mat& in) {
    cv::Mat out;
    cv::rotate(in, out, option);
    return out;
}

sensor_msgs::CameraInfo Rotate::apply(const sensor_msgs::CameraInfo& in) {
    sensor_msgs::CameraInfo out = in;
    switch (option) {
        case cv::RotateFlags::ROTATE_90_CLOCKWISE:
            // Swap width/height, fx/fy
            out.height = in.width;
            out.width = in.height;
            out.K[0] = in.K[4];
            out.K[4] = in.K[0];
            // new cx changed
            out.K[2] = in.height - in.K[5];
            // new cy = old cx
            out.K[5] = in.K[2];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 90 CW rotation.");
            break;
        case cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE:
            // Swap width/height, fx/fy
            out.height = in.width;
            out.width = in.height;
            out.K[0] = in.K[4];
            out.K[4] = in.K[0];
            // new cx = old cy
            out.K[2] = in.K[5];
            // new cy changed
            out.K[5] = in.width - in.K[2];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 90 CCW rotation.");
            break;
        case cv::RotateFlags::ROTATE_180:
            // new cx changed
            out.K[2] = in.width - in.K[2];
            // new cy changed
            out.K[5] = in.height - in.K[5];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 180 rotation.");
            break;
    }
    return out;
}

Flip::Flip(const Option option) {
    switch (option) {
        case Option::FLIP_HORIZ:
            flip_code = 1;
            break;
        case Option::FLIP_VERT:
            flip_code = 0;
            break;
        case Option::FLIP_BOTH:
            flip_code = -1;
            break;
        default:
            throw std::runtime_error("Flip option not recognised");
    }
}

cv::Mat Flip::apply(const cv::Mat& in) {
    cv::Mat out;
    cv::flip(in, out, flip_code);
    return out;
}

sensor_msgs::CameraInfo Flip::apply(const sensor_msgs::CameraInfo& in) {
    sensor_msgs::CameraInfo out = in;
    switch (flip_code) {
        case 1:  // horiz
            // cx flips
            out.K[2] = in.width - in.K[2];
            break;
        case 0:  // vert
            // cy flips
            out.K[5] = in.height - in.K[5];
            break;
        case -1:  // both
            // both cx/cy flip
            out.K[2] = in.width - in.K[2];
            out.K[5] = in.height - in.K[5];
            break;
    }
    ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after flip.");
    return out;
}

}
