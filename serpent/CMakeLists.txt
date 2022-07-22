cmake_minimum_required(VERSION 3.0)
project(serpent
    VERSION     1.0
    DESCRIPTION "State Estimation through Robust Perception in Extreme and Novel Terrains"
    LANGUAGES   CXX
)

option(ENABLE_GPROF "gprof profiling will run during execution" OFF)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    fast_gicp
    geometry_msgs
    image_transport
    message_filters
    message_generation
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_srvs
    tf2_ros

    eigen_ext
    eigen_gtsam
    eigen_ros
    pointcloud_tools
)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED) # >=4.4.0 required for SIFT
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
        cv_bridge
        fast_gicp
        geometry_msgs
        image_transport
        message_filters
        message_runtime
        nav_msgs
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_srvs
        tf2_ros

        eigen_ext
        eigen_gtsam
        eigen_ros
        pointcloud_tools
    DEPENDS
        EIGEN3
        OpenCV
        PCL
)

set(NODE ${PROJECT_NAME}_node)

set(NODE_SRC_FILES
    src/graph_manager.cpp
    src/li_frontend.cpp
    src/mapping.cpp
    src/match_filters.cpp
    src/node.cpp
    src/optimisation.cpp
    src/pointcloud_filter.cpp
    src/pointcloud_formatter.cpp
    src/pointcloud_normal_estimation.cpp
    src/registration_methods.cpp
    src/registration.cpp
    src/sensor_point_types.cpp
    src/stereo_factor_finder.cpp
    src/stereo_feature_tracker.cpp
    src/utilities.cpp
)

set(NODE_SYSTEM_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(NODE_SYSTEM_LIBRARIES
    ${EIGEN3_LIBRARIES}
    gtsam
    ${OpenCV_LIBRARIES}
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
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${NODE} PUBLIC
    include
)

target_link_libraries(${NODE} PUBLIC
    ${NODE_SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
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
        src/graph_manager.cpp
        src/utilities.cpp
        test/graph_manager.cpp
        test/registration_degeneracy.cpp
        test/serpent.cpp
        test/test_main.cpp
        test/src/read_data.cpp
        test/src/test_data.cpp
        test/src/test_utils.cpp
    )
    target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
        ${NODE_SYSTEM_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
    target_include_directories(test_${PROJECT_NAME} PUBLIC
        include
        test/include
    )
    target_link_libraries(test_${PROJECT_NAME}
        ${NODE_SYSTEM_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

    set_target_properties(test_${PROJECT_NAME}
        PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
    )
endif(CATKIN_ENABLE_TESTING)
