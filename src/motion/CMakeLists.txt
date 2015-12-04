cmake_minimum_required(VERSION 2.8.3)
project(motion)

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

auv_pkg(
    INCLUDE_DIRS ..
    LIBRARIES motion_client
    AUVDEP libipc libauv
    MSGDEP std_msgs libauv
)

add_node(motion motion_server.cpp)

add_lib(motion_client
      motion_client/mark_motion_client.cpp
      motion_client/motion_client.cpp
      motion_client/robosub_conf_motion_client.cpp
      motion_client/robosub_motion_client.cpp
)
