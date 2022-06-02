cmake_minimum_required(VERSION 3.0)
project(imu_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    sensor_msgs
    tf2_ros

    eigen_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
        sensor_msgs
        tf2_ros

        eigen_ros
    DEPENDS
        EIGEN3
)

# imu analysis
add_executable(imu_analysis
    src/imu_analysis_main.cpp
    src/imu_analysis.cpp
)

target_include_directories(imu_analysis PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(imu_analysis
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

# imu transform
add_executable(imu_transform
    src/imu_transform_main.cpp
    src/imu_transform.cpp
)

target_include_directories(imu_transform PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(imu_transform
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
