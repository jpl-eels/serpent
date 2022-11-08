#ifndef EIGEN_ROS_STD_MSGS_HPP
#define EIGEN_ROS_STD_MSGS_HPP

#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>

namespace eigen_ros {

template<typename Derived>
void from_ros(const std_msgs::Float64MultiArray& msg, Eigen::MatrixBase<Derived>& matrix);

template<typename Derived>
void to_ros(std_msgs::Float64MultiArray& msg, const Eigen::MatrixBase<Derived>& matrix);

/* Implementation ****************************************************************************************************/

template<typename Derived>
void from_ros(const std_msgs::Float64MultiArray& msg, Eigen::MatrixBase<Derived>& matrix) {
    if (msg.layout.dim.size() != 2 || msg.layout.dim[0].size != matrix.rows() ||
            msg.layout.dim[1].size != matrix.cols()) {
        throw std::runtime_error("Float64MultiArray msg dimensions did not match that of the matrix (" +
                                 std::to_string(matrix.rows()) + ", " + std::to_string(matrix.cols()));
    }
    for (int r = 0; r < msg.layout.dim[0].size; ++r) {
        for (int c = 0; c < msg.layout.dim[1].size; ++c) {
            matrix(r, c) = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride * r + c];
        }
    }
}

template<typename Derived>
void to_ros(std_msgs::Float64MultiArray& msg, const Eigen::MatrixBase<Derived>& matrix) {
    const int size = matrix.size();
    const int rows = matrix.rows();
    const int cols = matrix.cols();
    msg.data.resize(matrix.size());
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            msg.data[r * cols + c] = matrix(r, c);
        }
    }
    msg.layout.data_offset = 0;
    std_msgs::MultiArrayDimension row_dim;
    row_dim.label = "rows";
    row_dim.size = rows;
    row_dim.stride = size;  // dim[0] stride = size
    msg.layout.dim.push_back(row_dim);
    std_msgs::MultiArrayDimension col_dim;
    col_dim.label = "cols";
    col_dim.size = cols;
    col_dim.stride = cols;
    msg.layout.dim.push_back(col_dim);
}
}

#endif
