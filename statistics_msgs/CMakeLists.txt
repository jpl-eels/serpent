cmake_minimum_required(VERSION 3.0)
project(statistics_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    DIRECTORY msg
    FILES
        Covariance.msg
        CovarianceFloat32.msg
        CovarianceFloat32Stamped.msg
        CovarianceStamped.msg
        SummaryStatistics.msg
        SummaryStatisticsStamped.msg
        SummaryStatisticsArray.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    DEPENDS
)
