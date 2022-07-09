#include "eigen_ros/body_frames.hpp"
#include "eigen_ext/geometry.hpp"

namespace eigen_ros {

BodyFrames::BodyFrames():
    nh("~")
{
    XmlRpc::XmlRpcValue body_frames = nh.param<XmlRpc::XmlRpcValue>("body_frames", XmlRpc::XmlRpcValue{});
    if (body_frames.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("body_frames param was not a struct");
    }
    for (XmlRpc::XmlRpcValue::const_iterator it = body_frames.begin(); it != body_frames.end(); ++it) {
        const std::string frame = it->first;
        Eigen::Translation<double, 3> translation{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        if (it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            if (it->second.hasMember("translation")) {
                XmlRpc::XmlRpcValue translation_param = it->second["translation"];
                translation.x() = translation_param["x"];
                translation.y() = translation_param["y"];
                translation.z() = translation_param["z"];
            }
            if (it->second.hasMember("rotation")) {
                XmlRpc::XmlRpcValue rotation_param = it->second["rotation"];
                rotation.w() = rotation_param["w"];
                rotation.x() = rotation_param["x"];
                rotation.y() = rotation_param["y"];
                rotation.z() = rotation_param["z"];
            }
            const double rotation_norm = rotation.norm();
            if (std::abs(rotation_norm - 1.0) > 0.01) {
                ROS_WARN_STREAM(frame << " frame rotation was not normalised (|q| = " << rotation_norm << "). It will"
                        " be normalised.");
            }
            rotation.normalize();
        } else {
            throw std::runtime_error("body frame \'" + frame + "\' param was not a struct");
        }
        const Eigen::Isometry3d transform = eigen_ext::to_transform(translation, rotation);
        if (!body_to_frame_map.emplace(frame, transform).second) {
            throw std::runtime_error("failed to add body-to-frame transform for frame \'" + frame + "\'");
        }
        if (!frame_to_body_map.emplace(frame, transform.inverse()).second) {
            throw std::runtime_error("failed to add frame-to-body transform for frame \'" + frame + "\'");
        }
    }
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

}
