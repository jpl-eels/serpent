#include <ros/ros.h>

#include "camera_tools/change_frame_id.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "change_frame_id");
    camera_tools::ChangeFrameId change_frame_id;
    ros::spin();
    return 0;
}
