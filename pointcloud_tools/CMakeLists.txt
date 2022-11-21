cmake_minimum_required(VERSION 3.0)
project(pointcloud_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    message_generation
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros

    eigen_ext
    eigen_ros
    statistics_msgs
)
find_package(PCL REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
        file_to_message.srv
        message_to_file.srv
)

generate_messages(
    # DEPENDENCIES
)

set(LIBRARY_NAME ${PROJECT_NAME})

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        message_runtime
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        tf2_ros

        eigen_ext
        eigen_ros
        statistics_msgs
    LIBRARIES ${LIBRARY_NAME}
    DEPENDS
        PCL
)

set(SYSTEM_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(INCLUDE_DIRS
    include
)

set(LIBRARIES
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Library
add_library(${LIBRARY_NAME}
    src/pclpointcloud2_utilities.cpp
)

add_dependencies(${LIBRARY_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${LIBRARY_NAME} PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    ${INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
    ${LIBRARIES}
)

set(LIBRARIES ${LIBRARIES}
    ${LIBRARY_NAME}
)

# File converter
add_executable(pointcloud_file_converter
    src/pointcloud_file_converter_main.cpp
    src/pointcloud_file_converter.cpp
)

add_dependencies(pointcloud_file_converter
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

target_include_directories(pointcloud_file_converter PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)

target_include_directories(pointcloud_file_converter PUBLIC
    ${INCLUDE_DIRS}
)

target_link_libraries(pointcloud_file_converter
    ${LIBRARIES}
)

# Analyser
add_executable(pointcloud_analyser
    src/pointcloud_analyser_main.cpp
    src/pointcloud_analyser.cpp
)

add_dependencies(pointcloud_analyser
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(pointcloud_analyser PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)

target_include_directories(pointcloud_analyser PUBLIC
    ${INCLUDE_DIRS}
)

target_link_libraries(pointcloud_analyser
    ${LIBRARIES}
)

# ComputeDistances
add_executable(compute_distances
    src/compute_distances_main.cpp
)

add_dependencies(compute_distances
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(compute_distances PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)

target_include_directories(compute_distances PUBLIC
    ${INCLUDE_DIRS}
)

target_link_libraries(compute_distances
    ${LIBRARIES}
)
