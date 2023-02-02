#include "camera_tools/change_frame_id.hpp"

namespace camera_tools {

ChangeFrameId::ChangeFrameId()
    : nh("~"),
      it(nh),
      camera_sync(10) {
    // Subscribers
    image_subcriber.subscribe(nh, "input/image", 10);
    camera_info_subcriber.subscribe(nh, "input/camera_info", 10);
    camera_sync.connectInput(image_subcriber, camera_info_subcriber);
    camera_sync.registerCallback(boost::bind(&ChangeFrameId::callback, this, _1, _2));

    // Publishers
    image_publisher = it.advertise("output/image", 1);
    camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>("output/camera_info", 1);

    // Configuration
    nh.param<std::string>("frame_id", frame_id, "CHANGED_FRAME_ID_DEFAULT");
}

void ChangeFrameId::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& info) {
    auto new_image = boost::make_shared<sensor_msgs::Image>(*image);
    auto new_info = boost::make_shared<sensor_msgs::CameraInfo>(*info);
    new_image->header.frame_id = frame_id;
    new_info->header.frame_id = frame_id;
    image_publisher.publish(new_image);
    camera_info_publisher.publish(new_info);
}

}
