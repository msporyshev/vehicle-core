cmake_minimum_required(VERSION 2.8.3)
project(dvl)

find_package(Boost REQUIRED COMPONENTS system program_options)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
    MSGDIR msg
    MSGDEP std_msgs
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(dvl_node ${SOURCES})

target_link_libraries(dvl_node
    ${Boost_LIBRARIES}
)