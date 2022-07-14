#include "eigen_ros/body_frames_tf.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "body_frames_tf");
    eigen_ros::BodyFramesTf body_frames_tf;
    ros::spin();
    return 0;
}
