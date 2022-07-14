#ifndef EIGEN_ROS_BODY_FRAMES_HPP
#define EIGEN_ROS_BODY_FRAMES_HPP

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <string>

namespace eigen_ros {

/**
 * @brief Get the transform from a node of format:
 *      my_frame:
 *          translation:
 *              x: <double>
 *              y: <double>
 *              z: <double>
 *          rotation:
 *              w: <double>
 *              x: <double>
 *              y: <double>
 *              z: <double>
 * 
 * Note that each of x, y and z in the translation field are optional, and is replaced by 0.0 if missing.
 * The rotation field is also optional, and set to identity if missing. However if rotation exists, then all of w, x, y
 * and z must be set.  
 * 
 * @param node 
 * @return Eigen::Isometry3d 
 */
Eigen::Isometry3d transform_from_node(const XmlRpc::XmlRpcValue& node);

class BodyFrames {
public:
    /**
     * @brief BodyFrames reads from the ros-parameter server, looking for reference frames listed under body_frames in
     * the following format:
     * 
     *  body_frames:
     *      body_frame_name:
     *          my_named_frame:
     *              translation:
     *                  x: <double>
     *                  y: <double>
     *                  z: <double>
     *              rotation:
     *                  w: <double>
     *                  x: <double>
     *                  y: <double>
     *                  z: <double>
     *          ...
     */
    explicit BodyFrames();

    /**
     * @brief Get name of body frame
     * 
     * @return std::string 
     */
    std::string body_frame() const;

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

    /**
     * @brief Get a list of all frames
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> frames() const;

private:
    void process_frames(const XmlRpc::XmlRpcValue& frames, const Eigen::Isometry3d& prepend_transform);

    // ROS nodehandle
    ros::NodeHandle nh;

    std::string body_frame_;

    // Body-to-frame transforms
    std::map<std::string, Eigen::Isometry3d> body_to_frame_map;

    // Frame-to-body transforms (inverse)
    std::map<std::string, Eigen::Isometry3d> frame_to_body_map;
};

}

#endif
