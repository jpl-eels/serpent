cmake_minimum_required(VERSION 3.0)
project(navigation_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf2_ros

    eigen_ext
    eigen_gtsam
    eigen_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2_ros

        eigen_ext
        eigen_gtsam
        eigen_ros
)
find_package(GTSAM REQUIRED)

# combine_transforms
add_executable(combine_transforms
    src/combine_transforms_node.cpp
    src/combine_transforms.cpp
)

target_include_directories(combine_transforms PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(combine_transforms
    ${catkin_LIBRARIES}
)

# integrate imu
add_executable(integrate_imu
    src/integrate_imu_node.cpp
    src/integrate_imu.cpp
)

target_include_directories(integrate_imu PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(integrate_imu SYSTEM PUBLIC
    ${GTSAM_INCLUDE_DIRS}
)

target_link_libraries(integrate_imu
    ${catkin_LIBRARIES}
    gtsam
)
