cmake_minimum_required(VERSION 2.8.3)
project(motion)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS libipc libauv roscpp message_generation)

add_message_files(
    FILES
    MsgCmdStatus.msg
    MsgRegul.msg
    CmdFixDepth.msg
    CmdFixDepthConf.msg
    CmdFixHeading.msg
    CmdFixHeadingConf.msg
    CmdFixPitch.msg
    CmdFixPitchConf.msg
    CmdFixPosition.msg
    CmdFixPositionConf.msg
    CmdFixThrust.msg
    CmdFixVelocity.msg
    CmdFixVert.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES libipc
    CATKIN_DEPENDS message_runtime libipc
)

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(libmotion_client
      motion_client/mark_motion_client.cpp
      motion_client/motion_client.cpp
      motion_client/robosub_conf_motion_client.cpp
      motion_client/robosub_motion_client.cpp
)

## Declare a C++ executable
add_executable(motion motion_server.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(motion
    libipc
    ${catkin_LIBRARIES}
)
