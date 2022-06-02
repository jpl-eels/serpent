cmake_minimum_required(VERSION 3.0)
project(serpent
    VERSION     1.0
    DESCRIPTION "State Estimation through Robust Perception in Extreme and Novel Terrains"
    LANGUAGES   CXX
)

option(ENABLE_GPROF "gprof profiling will run during execution" OFF)

find_package(catkin REQUIRED COMPONENTS
    fast_gicp
    geometry_msgs
    message_filters
    message_generation
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2_ros

    eigen_ros
    pointcloud_tools
)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
    DIRECTORY msg
    FILES
        ImuArray.msg
        ImuBiases.msg
)

generate_messages(
    DEPENDENCIES
        sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS
        fast_gicp
        geometry_msgs
        message_filters
        message_runtime
        nav_msgs
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        tf2_ros

        eigen_ros
        pointcloud_tools
    DEPENDS
        EIGEN3
        PCL
)

set(NODE ${PROJECT_NAME}_node)

set(NODE_SRC_FILES
    src/li_frontend.cpp
    src/mapping.cpp
    src/node.cpp
    src/optimisation.cpp
    src/pointcloud_filter.cpp
    src/pointcloud_formatter.cpp
    src/pointcloud_normal_estimation.cpp
    src/registration_methods.cpp
    src/registration.cpp
    src/sensor_point_types.cpp
    src/utilities.cpp
)

set(NODE_SYSTEM_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

set(NODE_SYSTEM_LIBRARIES
    ${EIGEN3_LIBRARIES}
    gtsam
    ${PCL_LIBRARIES}
)

if(${ENABLE_GPROF})
    message(WARNING "gprof enabled - this should only be active while debugging")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif()

add_executable(${NODE}
    ${NODE_SRC_FILES}
)

add_dependencies(${NODE}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${NODE} SYSTEM PUBLIC
    ${NODE_SYSTEM_INCLUDE_DIRS}
)

target_include_directories(${NODE} PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${NODE} PUBLIC
    ${catkin_LIBRARIES}
    ${NODE_SYSTEM_LIBRARIES}
)

set_target_properties(${NODE}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Required (before PCL headers) from PCL 1.7 onwards when using custom point types to include templated algorithms also
target_compile_definitions(${NODE} PUBLIC PCL_NO_PRECOMPILE)

# Testing
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_${PROJECT_NAME}
        src/utilities.cpp
        test/serpent.cpp
        test/src/read_data.cpp
    )
    target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
        ${NODE_SYSTEM_INCLUDE_DIRS}
    )
    target_include_directories(test_${PROJECT_NAME} PUBLIC
        include
        test/include
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${NODE_SYSTEM_LIBRARIES}
    )
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif(CATKIN_ENABLE_TESTING)
