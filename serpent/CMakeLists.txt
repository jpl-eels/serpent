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

# GTSAM_42_* variables are set by core_config 
# They point to core_config's source build of gtsam required by serpent
find_package(GTSAM 4.2.0 EXACT REQUIRED PATHS $ENV{GTSAM_42_PATH})
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) # >=4.4.0 required for SIFT
find_package(PCL REQUIRED)

# OpenMP required to compile fast_gicp for new point types.
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_python_setup()

add_message_files(
    DIRECTORY msg
    FILES
        ImuArray.msg
        ImuBiases.msg
        PoseGraph.msg
        StereoFeature.msg
        StereoFeatures.msg
        StereoTrackerStatistics.msg
)

generate_messages(
    DEPENDENCIES
        sensor_msgs
        std_msgs
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

set(LIBRARY ${PROJECT_NAME})
set(NODE ${PROJECT_NAME}_node)
set(FRONTEND_NODE ${PROJECT_NAME}_frontend_node)
set(BACKEND_NODE ${PROJECT_NAME}_backend_node)
set(STEREO_PIPELINE_NODE stereo_pipeline_node)

set(SYSTEM_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(SYSTEM_LIBRARIES
    ${EIGEN3_LIBRARIES}
    gtsam
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Library
file(GLOB_RECURSE front_end_srcs "src/front_end/*.cpp")
file(GLOB_RECURSE back_end_srcs "src/back_end/*.cpp")
file(GLOB_RECURSE mapping_srcs "src/mapping/*.cpp")
file(GLOB_RECURSE utils_srcs "src/utils/*.cpp")
add_library(${LIBRARY} SHARED 
    ${front_end_srcs}
    ${back_end_srcs}
    ${mapping_srcs}
    ${utils_srcs}
)

if(${ENABLE_GPROF})
    message(WARNING "gprof enabled - this should only be active while debugging")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
endif()

add_dependencies(${LIBRARY}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${LIBRARY} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${LIBRARY} PUBLIC
    include
)

target_link_libraries(${LIBRARY} PUBLIC
    ${SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
)

set_target_properties(${LIBRARY}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Complete node
add_executable(${NODE}
    src/nodes/node.cpp
)

add_dependencies(${NODE}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${NODE} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${NODE} PUBLIC
    include
)

target_link_libraries(${NODE} PUBLIC
    ${LIBRARY}
    ${SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
)

set_target_properties(${NODE}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Frontend node
add_executable(${FRONTEND_NODE}
    src/nodes/frontend_node.cpp
)

add_dependencies(${FRONTEND_NODE}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${FRONTEND_NODE} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${FRONTEND_NODE} PUBLIC
    include
)

target_link_libraries(${FRONTEND_NODE} PUBLIC
    ${LIBRARY}
    ${SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
)

set_target_properties(${FRONTEND_NODE}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Backend node
add_executable(${BACKEND_NODE}
    src/nodes/backend_node.cpp
)

add_dependencies(${BACKEND_NODE}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${BACKEND_NODE} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${BACKEND_NODE} PUBLIC
    include
)

target_link_libraries(${BACKEND_NODE} PUBLIC
    ${LIBRARY}
    ${SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
)

set_target_properties(${BACKEND_NODE}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Stereo pipeline node
add_executable(${STEREO_PIPELINE_NODE}
    src/nodes/stereo_pipeline_node.cpp
)

add_dependencies(${STEREO_PIPELINE_NODE}
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${STEREO_PIPELINE_NODE} SYSTEM PUBLIC
    ${SYSTEM_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(${STEREO_PIPELINE_NODE} PUBLIC
    include
)

target_link_libraries(${STEREO_PIPELINE_NODE} PUBLIC
    ${LIBRARY}
    ${SYSTEM_LIBRARIES}
    ${catkin_LIBRARIES}
)

set_target_properties(${STEREO_PIPELINE_NODE}
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

# Required (before PCL headers) from PCL 1.7 onwards when using custom point types to include templated algorithms also
target_compile_definitions(${LIBRARY} PUBLIC PCL_NO_PRECOMPILE)
target_compile_definitions(${NODE} PUBLIC PCL_NO_PRECOMPILE)
target_compile_definitions(${FRONTEND_NODE} PUBLIC PCL_NO_PRECOMPILE)
target_compile_definitions(${BACKEND_NODE} PUBLIC PCL_NO_PRECOMPILE)
target_compile_definitions(${STEREO_PIPELINE_NODE} PUBLIC PCL_NO_PRECOMPILE)

# Testing
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_${PROJECT_NAME}
        src/back_end/graph_manager.cpp
        src/utils/utilities.cpp
        test/graph_manager.cpp
        test/registration_covariance.cpp
        test/serpent.cpp
        test/test_main.cpp
        test/src/read_data.cpp
        test/src/test_data.cpp
        test/src/test_utils.cpp
    )
    target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
        ${SYSTEM_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
    )
    target_include_directories(test_${PROJECT_NAME} PUBLIC
        include
        test/include
    )
    target_link_libraries(test_${PROJECT_NAME}
        ${SYSTEM_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

    set_target_properties(test_${PROJECT_NAME}
        PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
    )
endif(CATKIN_ENABLE_TESTING)
