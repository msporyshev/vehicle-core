cmake_minimum_required(VERSION 2.8.3)
project(sidesonar)

find_package(Boost REQUIRED COMPONENTS system program_options)

add_message_files(DIRECTORY msg FILES   MsgSidesonarLine.msg
                                        CmdSidesonarConfig.msg
                                        CmdSidesonarWorkStatus.msg)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(sidesonar_node ${SOURCES})

target_link_libraries(sidesonar_node
    ${Boost_LIBRARIES}
)