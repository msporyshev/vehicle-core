cmake_minimum_required(VERSION 2.8.3)
project(dvl)

find_package(Boost REQUIRED COMPONENTS system program_options)

add_message_files(DIRECTORY msg FILES   MsgDvlDistance.msg
                                        MsgDvlHeight.msg
                                        MsgDvlVelocity.msg)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(dvl_node ${SOURCES})

target_link_libraries(dvl_node
    ${Boost_LIBRARIES}
)