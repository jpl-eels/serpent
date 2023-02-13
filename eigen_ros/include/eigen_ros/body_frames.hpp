#ifndef EIGEN_ROS_BODY_FRAMES_HPP
#define EIGEN_ROS_BODY_FRAMES_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace eigen_ros {

class BodyFrames {
public:
    /**
     * @brief BodyFrames reads from the ros-parameter server, looking for reference frames listed under body_frames in
     * the format below. <> indicates replacement by the user which if a value is indicated below as <type: default>.
     * [] indicates optional.
     *
     *  body_frames:
     *      <body_frame_name>:
     *          [frame_id]: <string: body_frame_name>
     *          [frames]:
     *              <named_frame>:
     *                  [frame_id]: <string: named_frame>
     *                  [dynamic]: <bool: false>
     *                  [aliases]: <list(string): []>
     *                  [lookup_tf]: <bool: false>
     *                  [translation]:
     *                      x: <double: 0.0>
     *                      y: <double: 0.0>
     *                      z: <double: 0.0>
     *                  [rotation]:
     *                      w: <double: 1.0>
     *                      x: <double: 0.0>
     *                      y: <double: 0.0>
     *                      z: <double: 0.0>
     *                  [frames]:
     *                      ...
     *              ...
     *
     * Terminology:
     * * A "frame" is a string label for consistent internal usage (e.g. "imu", "lidar", "camera").
     * * A "frame_id" is the ROS frame_id used in std_msgs/Header messages and exists in the TF tree.
     * * An alias is also a frame_id, which allows multiple frame_ids to be associated to the same frame. Only used for
     *      static frames.
     *
     * Notes:
     * * The frame_id defaults to <named_frame> (or <body_frame_name>) if not set.
     * * dynamic == true is permitted only on leaf frames currently, and overrides aliases, lookup_tf, translation and
     *      rotation. When enabled, transforms to and from this frame require a TF2 lookup.
     * * If lookup_tf == true, then translation and rotation are ignored and the transform is found with a TF2 lookup
     *      from the parent frame_id to frame_id. Unless dynamic == true, this lookup is performed once.
     * * frames can have as many <named_frame> blocks as desired, and can be recursively nested.
     * * Aliases map to the same transform during lookup.
     */
    explicit BodyFrames();

    /**
     * @brief Return the aliases for a frame_id
     *
     * @param frame
     * @return std::vector<std::string>
     */
    std::vector<std::string> aliases(const std::string& frame_id) const;

    /**
     * @brief Get name of body frame <body_frame_name>.
     *
     * @return std::string
     */
    std::string body_frame() const;

    /**
     * @brief Get frame_id for body frame
     *
     * @return std::string
     */
    std::string body_frame_id() const;

    /**
     * @brief Look up the transform from the body frame to the named frame, T_B^F. If the frame is dynamic, then the
     * transform at timestamp will be looked up.
     *
     * @param frame
     * @param timestamp
     * @return Eigen::Isometry3d
     */
    Eigen::Isometry3d body_to_frame(const std::string& frame, const ros::Time timestamp = ros::Time(0)) const;

    /**
     * @brief Get the frame id associated with a frame. If no frame_id field was specified in the config, then the
     * name is the same.
     *
     * @param frame
     * @return std::string
     */
    std::string frame_id(const std::string& frame) const;

    /**
     * @brief Look up the transform from the named frame to the body frame, T_F^B. This is equivalent to the inverse of
     * the body-to-frame transform without requiring computation.
     *
     * @param frame
     * @return Eigen::Isometry3d
     */

    /**
     * @brief Look up the transform from the named frame to the body frame, T_F^B. This is equivalent to the inverse of
     * the body-to-frame transform without requiring computation. If the frame is dynamic, then the transform at
     * timestamp will be looked up.
     *
     * @param frame
     * @param timestamp
     * @return Eigen::Isometry3d
     */
    Eigen::Isometry3d frame_to_body(const std::string& frame, const ros::Time timestamp = ros::Time(0)) const;

    /**
     * @brief Compute the transform from frame1 to frame2.
     *
     *  T_1^2 = T_1^B * T_B^2
     *
     * @param frame1
     * @param frame2
     * @return Eigen::Isometry3d
     */

    /**
     * @brief Compute the transform from frame1 to frame2: T_1^2 = T_1^B * T_B^2. If either frame is dynamic, then the
     * transform at timestamp will be looked up.
     *
     * @param frame1
     * @param frame2
     * @param timestamp
     * @return Eigen::Isometry3d
     */
    Eigen::Isometry3d frame_to_frame(const std::string& frame1, const std::string& frame2,
            const ros::Time timestamp = ros::Time(0)) const;

    /**
     * @brief Get a list of all frames, including the body_frame.
     *
     * @return std::vector<std::string>
     */
    std::vector<std::string> frames() const;

    /**
     * @brief Get the frame for a known frame_id.
     *
     * @param frame_id
     * @return std::string
     */
    std::string frame(const std::string& frame_id) const;

    /**
     * @brief Check if frame exists.
     * 
     * @param frame 
     * @return true if frame exists
     * @return false otherwise
     */
    bool has_frame(const std::string& frame) const;

    /**
     * @brief Check if frame was looked up using TF api.
     *
     * @param frame
     * @return true if found using TF
     * @return false otherwise
     */
    bool lookedup_tf(const std::string& frame) const;

    /**
     * @brief Check if a frame is dynamic
     *
     * @param frame
     * @return true if frame is dynamic (requiring TF2 lookup)
     * @return false if frame is static
     */
    bool is_dynamic(const std::string& frame) const;

private:
    void add_frame_transforms(const std::string& frame, const Eigen::Isometry3d& body_to_frame);

    void add_frame_transforms(const std::string& frame, const Eigen::Isometry3d& body_to_frame,
            const Eigen::Isometry3d& frame_to_body);

    void add_frame_strings(const std::string& frame, const std::string& frame_id);

    /**
     * @brief Look up transform between two frames T_{frame1}^{frame2} at a particular timestamp using the TF2 API.
     * 
     * @param frame1 
     * @param frame2 
     * @param timestamp 
     * @return Eigen::Isometry3d 
     */
    Eigen::Isometry3d lookup_transform(const std::string& frame1, const std::string& frame2,
            const ros::Time timestamp) const;

    void process_frames(const XmlRpc::XmlRpcValue& frames, const Eigen::Isometry3d& prepend_transform,
            const std::string& parent_frame);

    /**
     * @brief Get the transform from a node of format:
     *      my_frame:
     *          lookup_tf: <bool>
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
     * The rotation field is also optional, and set to identity if missing. However if rotation exists, then all of w,
     * x, y and z must be set.
     *
     * @param node
     * @return Eigen::Isometry3d
     */
    Eigen::Isometry3d transform_from_node(const XmlRpc::XmlRpcValue& node, const std::string& frame,
            const std::string& parent_frame);

    //// ROS Communications
    // ROS nodehandle
    ros::NodeHandle nh;
    // TF2 Lookup
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    ros::Duration lookup_tf_timeout;

    // Body frame name
    std::string body_frame_;

    // frame -> frame_id
    std::unordered_map<std::string, std::string> frame_ids_map;

    // frame_id & aliases -> frame
    std::unordered_map<std::string, std::string> frames_map;

    // frame -> true if looked up from TF, false otherwise
    std::unordered_map<std::string, bool> lookedup_tf_map;

    // frame -> true if dynamic
    std::unordered_map<std::string, bool> dynamic_map;

    // frame_id -> aliases
    std::unordered_map<std::string, std::vector<std::string>> aliases_map;

    // frame -> body-to-frame transform
    std::unordered_map<std::string, Eigen::Isometry3d> body_to_frame_map;

    // frame -> frame-to-body transform (inverse of body-to-frame transform)
    std::unordered_map<std::string, Eigen::Isometry3d> frame_to_body_map;
};

}

#endif
