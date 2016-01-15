cmake_minimum_required(VERSION 2.8.3)
project(navig)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(
    cfg/LocalPositionEstimator.cfg
)

auv_pkg(
    INCLUDE_DIRS include
    ROSDEP dynamic_reconfigure
    AUVDEP libauv libipc
    MSGDIR msg
)

file(GLOB_RECURSE SOURCES "src/navig_node/*.cpp")
add_node(navig_node ${SOURCES})

file(GLOB_RECURSE SOURCES "src/local_position_estimator_node/*.cpp")
add_node(local_position_estimator_node ${SOURCES})

add_dependencies(local_position_estimator_node ${PROJECT_NAME}_gencfg)
