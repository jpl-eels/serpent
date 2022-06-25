cmake_minimum_required(VERSION 3.0)
project(eigen_ext)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    DEPENDS
        EIGEN3
)

# Testing
enable_testing()
if(CATKIN_ENABLE_TESTING)
    set(TEST_NAME test_eigen_ext)
    catkin_add_gtest(${TEST_NAME}
        test/covariance.cpp
        test/geometry.cpp
        test/main.cpp
        test/matrix.cpp
    )
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_include_directories(${TEST_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(${TEST_NAME}
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
    )
endif(CATKIN_ENABLE_TESTING)
