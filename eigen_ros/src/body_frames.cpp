#include "eigen_ros/body_frames.hpp"

#include <geometry_msgs/TransformStamped.h>

#include "eigen_ext/geometry.hpp"
#include "eigen_ros/geometry_msgs.hpp"

namespace eigen_ros {

BodyFrames::BodyFrames()
    : nh("~"),
      tf_listener(tf_buffer) {
    // TF lookup configuration
    lookup_tf_timeout = ros::Duration(nh.param<double>("lookup_tf_timeout", 1.0));

    // Body Frame
    XmlRpc::XmlRpcValue body_frames = nh.param<XmlRpc::XmlRpcValue>("body_frames", XmlRpc::XmlRpcValue{});
    if (body_frames.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("body_frames param was not a struct");
    } else if (body_frames.size() != 1) {
        throw std::runtime_error("expected one entry, the body frame, but found " + std::to_string(body_frames.size()));
    }
    XmlRpc::XmlRpcValue::const_iterator body_frame_it = body_frames.begin();
    body_frame_ = body_frame_it->first;
    const std::string body_frame_id_ =
            body_frame_it->second.hasMember("frame_id") ? std::string(body_frame_it->second["frame_id"]) : body_frame_;
    add_frame(body_frame_, body_frame_id_, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    if (!aliases_map.emplace(body_frame_id_, std::vector<std::string>{}).second) {
        throw std::runtime_error("failed to add aliases for body frame_id \"" + body_frame_id_ + "\" to aliases map");
    }

    // Frames
    if (body_frame_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("body_frame was not a struct");
    }
    if (body_frame_it->second.hasMember("frames")) {
        process_frames(body_frame_it->second["frames"], Eigen::Isometry3d::Identity(), body_frame_);
    }
}

std::vector<std::string> BodyFrames::aliases(const std::string& frame_id_) const {
    return aliases_map.at(frame_id_);
}

std::string BodyFrames::body_frame() const {
    return body_frame_;
}

std::string BodyFrames::body_frame_id() const {
    return frame_id(body_frame());
}

Eigen::Isometry3d BodyFrames::body_to_frame(const std::string& frame_) const {
    return body_to_frame_map.at(frame_);
}

std::string BodyFrames::frame_id(const std::string& frame_) const {
    return frame_ids_map.at(frame_);
}

Eigen::Isometry3d BodyFrames::frame_to_body(const std::string& frame_) const {
    return frame_to_body_map.at(frame_);
}

Eigen::Isometry3d BodyFrames::frame_to_frame(const std::string& frame1, const std::string& frame2) const {
    return frame_to_body(frame1) * body_to_frame(frame2);
}

std::vector<std::string> BodyFrames::frames() const {
    std::vector<std::string> frames_;
    for (const auto& key_value : frame_ids_map) {
        frames_.push_back(key_value.first);
    }
    return frames_;
}

std::string BodyFrames::frame(const std::string& frame_id_) const {
    return frames_map.at(frame_id_);
}

bool BodyFrames::lookedup_tf(const std::string& frame_id_) const {
    if (frame_id_ == body_frame_id())
        throw std::runtime_error("No lookup_tf from body->body exists");
    return lookedup_tf_map.at(frame_id_);
}

void BodyFrames::add_frame(const std::string& frame_, const std::string& frame_id_,
        const Eigen::Isometry3d& body_to_frame) {
    add_frame(frame_, frame_id_, body_to_frame, body_to_frame.inverse());
}

void BodyFrames::add_frame(const std::string& frame_, const std::string& frame_id_,
        const Eigen::Isometry3d& body_to_frame, const Eigen::Isometry3d& frame_to_body) {
    if (!frame_ids_map.emplace(frame_, frame_id_).second) {
        throw std::runtime_error("failed to add frame_id \"" + frame_id_ + "\" for frame \"" + frame_ + "\"");
    }
    if (!frames_map.emplace(frame_id_, frame_).second) {
        throw std::runtime_error("failed to add frame \"" + frame_ + "\" for frame_id \"" + frame_id_ + "\"");
    }
    if (!body_to_frame_map.emplace(frame_, body_to_frame).second) {
        throw std::runtime_error("failed to add body-to-frame transform for frame \'" + frame_ + "\'");
    }
    if (!frame_to_body_map.emplace(frame_, frame_to_body).second) {
        throw std::runtime_error("failed to add frame-to-body transform for frame \'" + frame_ + "\'");
    }
}

void BodyFrames::process_frames(const XmlRpc::XmlRpcValue& frames_, const Eigen::Isometry3d& prepend_transform,
        const std::string& parent_frame) {
    // Process each frame consecutively
    for (XmlRpc::XmlRpcValue::const_iterator it = frames_.begin(); it != frames_.end(); ++it) {
        // Frame transform
        const std::string frame_ = it->first;
        const std::string frame_id_ = it->second.hasMember("frame_id") ? std::string(it->second["frame_id"]) : frame_;
        const Eigen::Isometry3d transform =
                prepend_transform * transform_from_node(it->second, frame_id_, frame_id(parent_frame));
        add_frame(frame_, frame_id_, transform);

        // Aliases
        std::vector<std::string> aliases_vec;
        if (it->second.hasMember("aliases")) {
            const XmlRpc::XmlRpcValue aliases_ = it->second["aliases"];
            if (aliases_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                throw std::runtime_error("expected aliases to by an array of strings, e.g. [\"alias1\", ...]");
            }
            for (int i = 0; i < aliases_.size(); ++i) {
                if (aliases_[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                    throw std::runtime_error("alias was not a string");
                }
                const std::string alias = aliases_[i];
                if (!frames_map.emplace(alias, frame_).second) {
                    throw std::runtime_error("failed to add frame \"" + frame_ + "\" for alias \"" + alias + "\"");
                }
                aliases_vec.push_back(alias);
            }
        }
        if (!aliases_map.emplace(frame_id_, aliases_vec).second) {
            throw std::runtime_error("failed to add aliases for frame_id \"" + frame_id_ + "\" to aliases map");
        }

        // Process nested frames
        if (it->second.hasMember("frames")) {
            process_frames(it->second["frames"], transform, frame_);
        }
    }
}

Eigen::Isometry3d BodyFrames::transform_from_node(const XmlRpc::XmlRpcValue& node, const std::string& frame_id_,
        const std::string& parent_frame_id) {
    if (node.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("node was not a struct");
    }
    Eigen::Isometry3d transform;
    const bool lookup_tf = node.hasMember("lookup_tf") && (bool)node["lookup_tf"] ? true : false;
    if (!lookedup_tf_map.emplace(frame_id_, lookup_tf).second) {
        throw std::runtime_error("failed to add frame_id \"" + frame_id_ + "\" to looked_up map");
    }
    if (lookup_tf) {
        // We must have target_frame = parent_frame_id, source_frame = frame_id_ in order to obtain the transform of
        // frame_id_ with respect to parent_frame_id (T_{parent}^{child}).
        const geometry_msgs::TransformStamped tf =
                tf_buffer.lookupTransform(parent_frame_id, frame_id_, ros::Time(0), lookup_tf_timeout);
        eigen_ros::from_ros(tf.transform, transform);
    } else {
        Eigen::Translation<double, 3> translation{Eigen::Vector3d::Zero()};
        if (node.hasMember("translation")) {
            XmlRpc::XmlRpcValue translation_node = node["translation"];
            if (translation_node.hasMember("x")) {
                translation.x() = translation_node["x"];
            }
            if (translation_node.hasMember("y")) {
                translation.y() = translation_node["y"];
            }
            if (translation_node.hasMember("z")) {
                translation.z() = translation_node["z"];
            }
        }
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        if (node.hasMember("rotation")) {
            XmlRpc::XmlRpcValue rotation_node = node["rotation"];
            rotation.w() = rotation_node["w"];
            rotation.x() = rotation_node["x"];
            rotation.y() = rotation_node["y"];
            rotation.z() = rotation_node["z"];
        }
        rotation.normalize();
        transform = eigen_ext::to_transform(translation, rotation);
    }
    return transform;
}

}
