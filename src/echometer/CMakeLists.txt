cmake_minimum_required(VERSION 2.8.3)
project(echometer)

find_package(Boost REQUIRED COMPONENTS system program_options)

add_message_files(DIRECTORY msg FILES   MsgEchometerHeight.msg
                                        MsgEchometerTemperature.msg)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(echometer_node ${SOURCES})

target_link_libraries(echometer_node
    ${Boost_LIBRARIES}
)