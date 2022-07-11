#include "camera_tools/split_image.hpp"
#include <cv_bridge/cv_bridge.h>

namespace camera_tools {

Rotate::Rotate(const cv::RotateFlags option):
    option(option) {}

Rotate::Rotate(int degrees_clockwise) {
    degrees_clockwise = degrees_clockwise % 360; 
    if (degrees_clockwise <= 0) {
        throw std::runtime_error("Rotate degrees_clockwise must be > 0");
    }
     if (degrees_clockwise == 90) {
        option = cv::RotateFlags::ROTATE_90_CLOCKWISE;
    } else if (degrees_clockwise == 180) {
        option = cv::RotateFlags::ROTATE_180;
    } else if (degrees_clockwise == 270) {
        option = cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE;
    }
}

cv::Mat Rotate::apply(const cv::Mat& in) {
    cv::Mat out;
    cv::rotate(in, out, option);
    return out;
}

sensor_msgs::CameraInfo Rotate::apply(const sensor_msgs::CameraInfo& in) {
    sensor_msgs::CameraInfo out = in;
    switch (option) {
        case cv::RotateFlags::ROTATE_90_CLOCKWISE:
            // Swap width/height, fx/fy
            out.height = in.width;
            out.width = in.height;
            out.K[0] = in.K[4];
            out.K[4] = in.K[0];
            // new cx changed
            out.K[2] = in.height - in.K[5];
            // new cy = old cx
            out.K[5] = in.K[2];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 90 CW rotation.");
            break;
        case cv::RotateFlags::ROTATE_90_COUNTERCLOCKWISE:
            // Swap width/height, fx/fy
            out.height = in.width;
            out.width = in.height;
            out.K[0] = in.K[4];
            out.K[4] = in.K[0];
            // new cx = old cy
            out.K[2] = in.K[5];
            // new cy changed
            out.K[5] = in.width - in.K[2];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 90 CCW rotation.");
            break;
        case cv::RotateFlags::ROTATE_180:
            // new cx changed
            out.K[2] = in.width - in.K[2];
            // new cy changed
            out.K[5] = in.height - in.K[5];
            ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after 180 rotation.");
            break;
    }
    return out;
}


Flip::Flip(const Option option){
    switch (option) {
        case Option::FLIP_HORIZ:
            flip_code = 1;
            break;
        case Option::FLIP_VERT:
            flip_code = 0;
            break;
        case Option::FLIP_BOTH:
            flip_code = -1;
            break;
        default:
            throw std::runtime_error("Flip option not recognised");
    }
}

cv::Mat Flip::apply(const cv::Mat& in) {
    cv::Mat out;
    cv::flip(in, out, flip_code);
    return out;
}

sensor_msgs::CameraInfo Flip::apply(const sensor_msgs::CameraInfo& in) {
    sensor_msgs::CameraInfo out = in;
    switch (flip_code) {
        case 1: // horiz
            // cx flips
            out.K[2] = in.width - in.K[2];
            break;
        case 0: // vert
            // cy flips
            out.K[5] = in.height - in.K[5];
            break;
        case -1: // both
            // both cx/cy flip
            out.K[2] = in.width - in.K[2];
            out.K[5] = in.height - in.K[5];
            break;
    }
    ROS_WARN_ONCE("CameraInfo D, R and P may be invalid after flip.");
    return out;
}

ImageSplitter::ImageSplitter():
    nh("~"), it(nh)
{
    image_subscriber = it.subscribe("input/image", 10, &ImageSplitter::split, this);
    parse_configuration();
}

void ImageSplitter::parse_configuration() {
    const XmlRpc::XmlRpcValue images = nh.param<XmlRpc::XmlRpcValue>("images", XmlRpc::XmlRpcValue{});
    if (images.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error("images param was not an array");
    }
    for (std::size_t i = 0; i < images.size(); ++i) {
        XmlRpc::XmlRpcValue image = images[i];
        if (image.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("images[" + std::to_string(i) + "] was not a struct");
        }
        const std::string topic = image["topic"];
        XmlRpc::XmlRpcValue region = image["region"];
        if (region.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("region for images[" + std::to_string(i) + "] was not a struct");
        }
        cv::Rect region_rect{region["u"], region["v"], region["w"], region["h"]};
        const std::string info_topic = image["info_topic"];
        sensor_msgs::CameraInfo camera_info;
        camera_info.height = region_rect.height;
        camera_info.width = region_rect.width;
        XmlRpc::XmlRpcValue distortion = image["distortion"];
        camera_info.distortion_model = std::string(distortion["model"]);
        camera_info.D.resize(5);
        camera_info.D[0] = distortion["k1"];
        camera_info.D[1] = distortion["k2"];
        camera_info.D[2] = distortion["t1"];
        camera_info.D[3] = distortion["t2"];
        camera_info.D[4] = distortion["k3"];
        XmlRpc::XmlRpcValue intrinsic = image["intrinsic"];
        camera_info.K[0] = intrinsic["fx"];
        camera_info.P[0] = camera_info.K[0];
        camera_info.K[2] = (int)intrinsic["cx"];
        camera_info.P[2] = camera_info.K[2];
        camera_info.K[4] = intrinsic["fy"];
        camera_info.P[5] = camera_info.K[4];
        camera_info.K[5] = (int)intrinsic["cy"];
        camera_info.P[6] = camera_info.K[5];
        camera_info.K[8] = 1.0;
        camera_info.P[10] = camera_info.K[8];
        ROS_WARN_ONCE("Camera info R matrices not set, P matrix missing Tx Ty for stereo");
        image_configs.emplace_back(ImageSplitConfig{.publisher = it.advertise(topic, 1),
                .info_publisher = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 1), .frame_id = image["frame_id"],
                .region = region_rect, .camera_info = camera_info});
        if (image.hasMember("operations")) {
            XmlRpc::XmlRpcValue operations = image["operations"];
            if (region.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                throw std::runtime_error("operations for images[" + std::to_string(i) + "] was not a struct");
            }
            for (XmlRpc::XmlRpcValue::const_iterator it = operations.begin(); it != operations.end(); ++it) {
                if (it->first == "rotate_cw") {
                    image_configs.back().operations.emplace_back(std::make_shared<Rotate>(it->second));
                } else if (it->first == "rotate_acw") {
                    image_configs.back().operations.emplace_back(std::make_shared<Rotate>((360-(int)it->second)%360));
                } else if (it->first == "flip") {
                    const std::string flip_direction = it->second;
                    if (flip_direction == "horiz") {
                        image_configs.back().operations.emplace_back(std::make_shared<Flip>(Flip::Option::FLIP_HORIZ));
                    } else if (flip_direction == "vert") {
                        image_configs.back().operations.emplace_back(std::make_shared<Flip>(Flip::Option::FLIP_VERT));
                    } else if (flip_direction == "both") {
                        image_configs.back().operations.emplace_back(std::make_shared<Flip>(Flip::Option::FLIP_BOTH));
                    } else {
                        throw std::runtime_error("flip direction \'" + flip_direction + "\' not recognised");
                    }
                } else {
                    throw std::runtime_error("Unrecognised operation \'" + it->first + "\'");
                }
            }   
        }
        ROS_INFO_STREAM("Publishing images on topic = " << topic << " with CameraInfo on topic = " << info_topic);
    }
}

void ImageSplitter::split(const sensor_msgs::ImageConstPtr& msg) {
    const ros::WallTime tic = ros::WallTime::now();
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg);
    for (const auto& image_config : image_configs) {
        cv_bridge::CvImage output{cv_image->header, cv_image->encoding, cv_image->image(image_config.region)};
        output.header.frame_id = image_config.frame_id;
        sensor_msgs::CameraInfo camera_info = image_config.camera_info;
        camera_info.header = output.header;
        for (const auto& operation : image_config.operations) {
            output.image = operation->apply(output.image);
            camera_info = operation->apply(camera_info);
        }
        image_config.publisher.publish(output.toImageMsg());
        image_config.info_publisher.publish(camera_info);
    }
    ROS_DEBUG_STREAM("Split and published images in " << (ros::WallTime::now() - tic).toSec() << " seconds.");
}

}
