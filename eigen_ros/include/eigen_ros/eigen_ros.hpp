#ifndef EIGEN_ROS_EIGEN_ROS_HPP
#define EIGEN_ROS_EIGEN_ROS_HPP

#include <ros/ros.h>

#include <Eigen/Geometry>

// Include first to use templated to_ros/from_ros convenience function
#include "eigen_ros/geometry_msgs.hpp"
#include "eigen_ros/nav_msgs.hpp"
#include "eigen_ros/sensor_msgs.hpp"
#include "eigen_ros/statistics_msgs.hpp"
#include "eigen_ros/std_msgs.hpp"

namespace eigen_ros {

Eigen::Matrix3d matrix3d_from_param(const ros::NodeHandle& nh, const std::string& param);

template<typename ArrayScalar, int Rows, int Cols, typename MatrixScalar = ArrayScalar>
void from_ros(const boost::array<ArrayScalar, Rows * Cols>& msg, Eigen::Matrix<MatrixScalar, Rows, Cols>& m) {
    for (std::size_t r = 0; r < Rows; ++r) {
        for (std::size_t c = 0; c < Cols; ++c) {
            m(r, c) = msg[Cols * r + c];
        }
    }
}

template<typename ArrayScalar, int Rows, int Cols, typename MatrixScalar = ArrayScalar>
void to_ros(boost::array<ArrayScalar, Rows * Cols>& msg, const Eigen::Matrix<MatrixScalar, Rows, Cols>& m) {
    for (std::size_t r = 0; r < Rows; ++r) {
        for (std::size_t c = 0; c < Cols; ++c) {
            msg[Cols * r + c] = m(r, c);
        }
    }
}

template<typename ArrayScalar, typename MatrixScalar = ArrayScalar>
void from_ros(const std::vector<ArrayScalar>& msg, Eigen::Matrix<MatrixScalar, Eigen::Dynamic, Eigen::Dynamic>& m) {
    std::size_t Rows = m.rows();
    std::size_t Cols = m.cols();
    if (msg.size() != Rows * Cols) {
        throw std::runtime_error("Failed to convert from ros vector to matrix. Vector size " +
                                 std::to_string(msg.size()) + " != " + std::to_string(Rows) + "*" +
                                 std::to_string(Cols) + " (Rows*Cols). Were the matrix dimensions set correctly?");
    }
    for (std::size_t r = 0; r < Rows; ++r) {
        for (std::size_t c = 0; c < Cols; ++c) {
            m(r, c) = msg[Cols * r + c];
        }
    }
}

template<typename ArrayScalar, typename MatrixScalar = ArrayScalar>
void to_ros(std::vector<ArrayScalar>& msg, const Eigen::Matrix<MatrixScalar, Eigen::Dynamic, Eigen::Dynamic>& m) {
    std::size_t Rows = m.rows();
    std::size_t Cols = m.cols();
    msg.resize(Rows * Cols);
    for (std::size_t r = 0; r < Rows; ++r) {
        for (std::size_t c = 0; c < Cols; ++c) {
            msg[Cols * r + c] = m(r, c);
        }
    }
}

/**
 * @brief Convenience function to convert from a ROS message type. Requires the following function to be included:
 *      void from_ros(const FromType&, ToType&);
 *
 * Example usage:
 *      MyType t = from_ros<MyType>(msg);
 *
 * @tparam ToType
 * @tparam FromType
 * @param msg
 * @return ToType
 */
template<typename ToType, typename FromType>
inline ToType from_ros(const FromType& msg) {
    ToType t;
    from_ros(msg, t);
    return t;
}

/**
 * @brief Convenience function to convert to a ROS message type. Requires the following function to be included:
 *      void to_ros(ToType&, const FromType&);
 *
 * Example usage:
 *      RosType msg = from_ros<RosType>(t);
 *
 * @tparam ToType
 * @tparam FromType
 * @param t
 * @return ToType
 */
template<typename ToType, typename FromType>
inline ToType to_ros(const FromType& t) {
    ToType msg;
    to_ros(msg, t);
    return msg;
}

/**
 * @brief Convenience function to convert to a stamped ROS message type.
 * 
 * @tparam ToType 
 * @tparam FromType 
 * @param t 
 * @param timestamp 
 * @param frame_id 
 * @return ToType 
 */
template<typename ToType, typename FromType>
inline ToType to_ros(const FromType& t, const ros::Time& timestamp, const std::string& frame_id) {
    ToType msg;
    to_ros(msg, t, timestamp, frame_id);
    return msg;
}

}

#endif
