cmake_minimum_required(VERSION 3.0)
project(camera_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
    DEPENDS
        EIGEN3
        OpenCV
)

# imu analysis
add_executable(split_image
    src/split_image_main.cpp
    src/split_image.cpp
)

target_include_directories(split_image PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(split_image
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
)