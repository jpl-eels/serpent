cmake_minimum_required(VERSION 3.0)
project(navigation_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf2_ros

    eigen_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        tf2_ros

        eigen_ros
)

# combine_transforms
add_executable(combine_transforms
    src/combine_transforms_main.cpp
    src/combine_transforms.cpp
)

target_include_directories(combine_transforms PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(combine_transforms
    ${catkin_LIBRARIES}
)
