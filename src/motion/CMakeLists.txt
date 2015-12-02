cmake_minimum_required(VERSION 2.8.3)
project(motion)

find_package(catkin REQUIRED COMPONENTS libipc libauv roscpp message_generation genmsg)

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
    libauv
)

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES libipc libauv
    CATKIN_DEPENDS message_runtime libipc libauv
)

include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(motion motion_server.cpp)

add_dependencies(motion ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
add_library(libmotion_client
      motion_client/mark_motion_client.cpp
      motion_client/motion_client.cpp
      motion_client/robosub_conf_motion_client.cpp
      motion_client/robosub_motion_client.cpp
)

add_dependencies(libmotion_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(motion
    libipc
    libauv
    ${catkin_LIBRARIES}
)
