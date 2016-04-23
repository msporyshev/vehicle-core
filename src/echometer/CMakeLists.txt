cmake_minimum_required(VERSION 2.8.3)
project(echometer)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
    MSGDEP std_msgs supervisor
    MSGDIR msg
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(echometer_node ${SOURCES})

target_link_libraries(echometer_node
    ${Boost_LIBRARIES}
)