cmake_minimum_required(VERSION 3.0)
project(eigen_gtsam)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

set(LIBRARY_NAME ${PROJECT_NAME})

catkin_package(
    INCLUDE_DIRS
        include
    DEPENDS
        EIGEN3
    LIBRARIES
        ${LIBRARY_NAME}
)

add_library(${LIBRARY_NAME}
    src/geometry.cpp
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
