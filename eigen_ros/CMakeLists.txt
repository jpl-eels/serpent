cmake_minimum_required(VERSION 3.0)
project(eigen_ros)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    tf2_ros

    eigen_ext
)
find_package(Eigen3 REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2_ros

        eigen_ext
    DEPENDS
        EIGEN3
    LIBRARIES
        ${LIBRARY_NAME}
)

add_library(${LIBRARY_NAME}
    src/body_frames.cpp
    src/eigen_ros.cpp
    src/geometry_msgs.cpp
    src/imu.cpp
    src/nav_msgs.cpp
    src/odometry.cpp
    src/pose.cpp
    src/sensor_msgs.cpp
    src/twist.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(${LIBRARY_NAME} PUBLIC SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

set_target_properties(${LIBRARY_NAME}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Testing
enable_testing()
if(CATKIN_ENABLE_TESTING)
    function(make_test TEST_FILE)
        set(TEST_NAME test_${TEST_FILE})
        catkin_add_gtest(${TEST_NAME}
            test/${TEST_FILE}.cpp
            test/src/test_instances.cpp
        )
        target_include_directories(${TEST_NAME} SYSTEM PUBLIC
            ${EIGEN3_INCLUDE_DIRS}
        )
        target_include_directories(${TEST_NAME} PUBLIC
            include
            test/include
            ${catkin_INCLUDE_DIRS}
        )
        target_link_libraries(${TEST_NAME}
            ${catkin_LIBRARIES}
            ${EIGEN3_LIBRARIES}
            ${LIBRARY_NAME}
        )
    endfunction()
    make_test(eigen_ros)
    make_test(geometry_msgs)
    make_test(nav_msgs)
    make_test(odometry)
    make_test(pose)
    make_test(sensor_msgs)
endif(CATKIN_ENABLE_TESTING)
