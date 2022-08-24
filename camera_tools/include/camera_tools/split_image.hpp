#ifndef CAMERA_TOOLS_SPLIT_IMAGE_HPP
#define CAMERA_TOOLS_SPLIT_IMAGE_HPP

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <opencv2/core/types.hpp>
#include <optional>

#include "camera_tools/image_operations.hpp"

namespace camera_tools {

struct ImageSplitConfig {
    image_transport::Publisher publisher;
    ros::Publisher info_publisher;
    std::string frame_id;
    cv::Rect region;
    sensor_msgs::CameraInfo camera_info;
    std::vector<std::shared_ptr<ImageOperation>> operations;
};

/**
 * @brief ROS node for splitting up or extracting regions from an image and republishing them as new image + camera_info
 * streams.
 *
 * Subscribes to ~input/image, which may be remapped in a launch file.
 *
 * Reads configuration from ROS parameter server. For example:
 *      images:
 *        - topic: "/camera1/image_raw"
 *          frame_id: "camera1"
 *          info_topic: "/camera1/camera_info"
 *          region:
 *            u: 0
 *            v: 0
 *            w: 800
 *            h: 600
 *          operations:
 *            rotate_cw: 180
 *            flip: "vert"
 *          distortion:
 *            model: "plumb_bob"
 *            k1: 0.0
 *            k2: 0.0
 *            k3: 0.0
 *            t1: 0.0
 *            t2: 0.0
 *          intrinsic:
 *            fx: 1.0
 *            fy: 1.0
 *            cx: 400
 *            cy: 300
 *        ...
 *
 * Valid operations:
 *  rotate_cw: <0, 90, 180, 270>
 *  rotate_acw: <0, 90, 180, 270>
 *  flip: "horiz", "vert", "both"
 *
 */
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
