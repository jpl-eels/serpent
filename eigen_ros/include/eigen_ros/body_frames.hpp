#ifndef EIGEN_ROS_BODY_FRAMES_HPP
#define EIGEN_ROS_BODY_FRAMES_HPP

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <string>

namespace eigen_ros {

class BodyFrames {
public:
    /**
     * @brief BodyFrames reads from the ros-parameter server, looking for reference frames listed under body_frames in
     * the following format:
     * 
     *  body_frames:
     *      my_named_frame:
     *          translation:
     *              x: <double>
     *              y: <double>
     *              z: <double>
     *          rotation:
     *              w: <double>
     *              x: <double>
     *              y: <double>
     *              z: <double>
     *      ...
     */
    explicit BodyFrames();

    /**
     * @brief Look up the transform from the body frame to the named frame, T_B^F.
     * 
     * @param frame 
     * @return Eigen::Isometry3d 
     */
    Eigen::Isometry3d body_to_frame(const std::string& frame) const;

    /**
     * @brief Look up the transform from the named frame to the body frame, T_F^B. This is equivalent to the inverse of
     * the body-to-frame transform without requiring computation.
     * 
     * @param frame 
     * @return Eigen::Isometry3d 
     */
    Eigen::Isometry3d frame_to_body(const std::string& frame) const;

    /**
     * @brief Compute the transform from frame1 to frame2.
     * 
     *  T_1^2 = T_1^B * T_B^2
     * 
     * @param frame1
     * @param frame2
     * @return Eigen::Isometry3d 
     */
    Eigen::Isometry3d frame_to_frame(const std::string& frame1, const std::string& frame2) const;

private:
    ros::NodeHandle nh;

    // Body-to-frame transforms
    std::map<std::string, Eigen::Isometry3d> body_to_frame_map;

    // Frame-to-body transforms (inverse)
    std::map<std::string, Eigen::Isometry3d> frame_to_body_map;
};

}

#endif
