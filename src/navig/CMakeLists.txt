cmake_minimum_required(VERSION 2.8.3)
project(navig)

# find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)
# find_package(Boost REQUIRED COMPONENTS program_options)

# generate_dynamic_reconfigure_options(
#     cfg/PositionEstimator.cfg
#     cfg/Navig.cfg
# )

auv_pkg(
    ROSDEP dynamic_reconfigure
    AUVDEP libauv libipc
    MSGDIR msg
    MSGDEP std_msgs dvl gps supervisor compass
)

add_node(navig "main.cpp")

# add_deps(navig)
# add_dependencies(navig ${PROJECT_NAME}_gencfg)

target_link_libraries(navig
    ${Boost_LIBRARIES}
)
