cmake_minimum_required(VERSION 2.8.3)
project(video)

add_message_files(FILES
    CmdSwitchCamera.msg
    MsgVideoFrame.msg
    MsgFoundBin.msg
    MsgBin.msg
)

auv_pkg(
    AUVDEP libauv libipc
    # MSGDEP sensor_msgs
)

add_node(video_node main.cpp)

