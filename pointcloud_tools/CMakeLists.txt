cmake_minimum_required(VERSION 3.0)
project(pointcloud_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    message_generation
    open3d_conversions
    pcl_conversions
    roscpp
    sensor_msgs
)
find_package(Open3D REQUIRED)

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
        open3d_conversions
        pcl_conversions
        roscpp
        sensor_msgs
    LIBRARIES ${LIBRARY_NAME}
    DEPENDS
        Open3D
        PCL
)

set(SYSTEM_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(INCLUDE_DIRS
    include
)

set(LIBRARIES
    ${catkin_LIBRARIES}
    ${Open3D_LIBRARIES}
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

# Range filter
add_executable(pointcloud_range_filter
    src/pointcloud_range_filter_main.cpp
    src/pointcloud_range_filter.cpp
)

add_dependencies(pointcloud_range_filter
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(pointcloud_range_filter PUBLIC SYSTEM
    ${SYSTEM_INCLUDE_DIRS}
)

target_include_directories(pointcloud_range_filter PUBLIC
    ${INCLUDE_DIRS}
)

target_link_libraries(pointcloud_range_filter
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
