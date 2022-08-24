#ifndef CAMERA_TOOLS_SPLIT_IMAGE_HPP
#define CAMERA_TOOLS_SPLIT_IMAGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace camera_tools {

/**
 * @brief ROS node class for publishing camera info messages for a given image stream.
 *
 * Subscribes to ~input/image, which may be remapped in a launch file.
 * Publishes to ~output/camera_info, which may be remapped in a launch file.
 *
 * Camera info parameters read from ROS parameter server. Monocular camera example:
 *      distortion_model: "plumb_bob"
 *      distortion_coefficients: []
 *      K: &K
 *       fx: 600
 *       fy: 400
 *       cx: 480
 *       cy: 320
 *      P: *K
 *
 * A stereo camera will need to set Tx, for example:
 *      P:
 *        fx: 600
 *        fy: 400
 *        cx: 480
 *        cy: 320
 *        Tx: -60.0
 */
class PublishInfo {
public:
    explicit PublishInfo();

private:
    void callback(const sensor_msgs::ImageConstPtr& msg);

    //// ROS Communications
    ros::NodeHandle nh;
    ros::Subscriber image_subscriber;
    ros::Publisher camera_info_publisher;

    // Camera Info
    sensor_msgs::CameraInfo camera_info;
};

}

#endif
