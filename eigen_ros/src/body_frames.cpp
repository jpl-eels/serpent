#include "eigen_ros/body_frames.hpp"

#include "eigen_ext/geometry.hpp"

namespace eigen_ros {

Eigen::Isometry3d transform_from_node(const XmlRpc::XmlRpcValue& node) {
    if (node.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("node was not a struct");
    }
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
    return eigen_ext::to_transform(translation, rotation);
}

BodyFrames::BodyFrames()
    : nh("~") {
    XmlRpc::XmlRpcValue body_frames = nh.param<XmlRpc::XmlRpcValue>("body_frames", XmlRpc::XmlRpcValue{});
    if (body_frames.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("body_frames param was not a struct");
    } else if (body_frames.size() != 1) {
        throw std::runtime_error("expected one entry, the body frame, but found " + std::to_string(body_frames.size()));
    }
    XmlRpc::XmlRpcValue::const_iterator body_frame_it = body_frames.begin();
    body_frame_ = body_frame_it->first;
    if (body_frame_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("body_frame was not a struct");
    }
    process_frames(body_frame_it->second, Eigen::Isometry3d::Identity());
}

std::string BodyFrames::body_frame() const {
    return body_frame_;
}

Eigen::Isometry3d BodyFrames::body_to_frame(const std::string& frame) const {
    return body_to_frame_map.at(frame);
}

Eigen::Isometry3d BodyFrames::frame_to_body(const std::string& frame) const {
    return frame_to_body_map.at(frame);
}

Eigen::Isometry3d BodyFrames::frame_to_frame(const std::string& frame1, const std::string& frame2) const {
    return frame_to_body(frame1) * body_to_frame(frame2);
}

std::vector<std::string> BodyFrames::frames() const {
    std::vector<std::string> frames_;
    for (const auto& key_value : body_to_frame_map) {
        frames_.push_back(key_value.first);
    }
    return frames_;
}

void BodyFrames::process_frames(const XmlRpc::XmlRpcValue& frames, const Eigen::Isometry3d& prepend_transform) {
    for (XmlRpc::XmlRpcValue::const_iterator it = frames.begin(); it != frames.end(); ++it) {
        const std::string frame = it->first;
        const Eigen::Isometry3d transform = prepend_transform * transform_from_node(it->second);
        if (!body_to_frame_map.emplace(frame, transform).second) {
            throw std::runtime_error("failed to add body-to-frame transform for frame \'" + frame + "\'");
        }
        if (!frame_to_body_map.emplace(frame, transform.inverse()).second) {
            throw std::runtime_error("failed to add frame-to-body transform for frame \'" + frame + "\'");
        }
        if (it->second.hasMember("frames")) {
            process_frames(it->second["frames"], transform);
        }
    }
}

}
