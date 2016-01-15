cmake_minimum_required(VERSION 2.8.3)
project(navig)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(
    cfg/LocalPositionEstimator.cfg
)

auv_pkg(
    ROSDEP dynamic_reconfigure
    AUVDEP libauv libipc
    MSGDIR msg
)

add_node(navig_node navig.cpp main.cpp)

add_node(local_position_estimator_node local_position_estimator/local_position_estimator.cpp local_position_estimator/dynamic_parameters.cpp local_position_estimator/main.cpp)

add_dependencies(local_position_estimator_node ${PROJECT_NAME}_gencfg)
