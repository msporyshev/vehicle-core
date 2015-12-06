cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(Boost REQUIRED COMPONENTS system program_options)

add_message_files(DIRECTORY msg FILES   MsgCameraFrame.msg
                                        CmdCameraConfig.msg)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(camera_node ${SOURCES})

target_link_libraries(camera_node
    ${Boost_LIBRARIES}
)