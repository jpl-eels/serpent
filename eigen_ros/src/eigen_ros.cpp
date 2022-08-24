#include "eigen_ros/eigen_ros.hpp"

namespace eigen_ros {

Eigen::Matrix3d matrix3d_from_param(const ros::NodeHandle& nh, const std::string& param) {
    Eigen::Matrix3d matrix;
    std::vector<double> matrix_vec = nh.param<std::vector<double>>(param, {{1, 0, 0, 0, 1, 0, 0, 0, 1}});
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            matrix(r, c) = matrix_vec.at(r * 3 + c);
        }
    }
    return matrix;
}

}
