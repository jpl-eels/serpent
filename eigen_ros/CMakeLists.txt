cmake_minimum_required(VERSION 3.0)
project(eigen_ros)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_ros

    eigen_ext
    statistics_msgs
)
find_package(Eigen3 REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
        lookup_transform.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
)

set(LIBRARY_NAME ${PROJECT_NAME})

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros

        eigen_ext
        statistics_msgs
    DEPENDS
        EIGEN3
    LIBRARIES
        ${LIBRARY_NAME}
)

# Library
add_library(${LIBRARY_NAME}
    src/body_frames_tf.cpp
    src/body_frames.cpp
    src/eigen_ros.cpp
    src/geometry_msgs.cpp
    src/imu.cpp
    src/nav_msgs.cpp
    src/odometry.cpp
    src/pose.cpp
    src/sensor_msgs.cpp
    src/std_msgs.cpp
    src/statistics_msgs.cpp
    src/twist.cpp
)

add_dependencies(${LIBRARY_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
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

# Executable
add_executable(body_frames_tf_node
    src/body_frames_tf_node.cpp
)

add_dependencies(body_frames_tf_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

target_include_directories(body_frames_tf_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(body_frames_tf_node PUBLIC SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(body_frames_tf_node
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

set_target_properties(body_frames_tf_node
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
    make_test(imu)
    make_test(nav_msgs)
    make_test(odometry)
    make_test(pose)
    make_test(sensor_msgs)
    make_test(statistics_msgs)
endif(CATKIN_ENABLE_TESTING)
