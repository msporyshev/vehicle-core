cmake_minimum_required(VERSION 2.8.3)
project(gps)

find_package(Boost REQUIRED COMPONENTS system program_options)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
    MSGDIR msg
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(gps_node ${SOURCES})



target_link_libraries(gps_node
    ${Boost_LIBRARIES}
)