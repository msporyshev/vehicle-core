cmake_minimum_required(VERSION 2.8.3)
project(motion)

auv_pkg(
    INCLUDE_DIRS ..
    LIBRARIES motion_client
    AUVDEP libipc libauv
    MSGDEP std_msgs libauv
    MSGDIR msg
)

add_node(motion motion_server.cpp)

add_lib(motion_client
      motion_client/mark_motion_client.cpp
      motion_client/motion_client.cpp
      motion_client/robosub_conf_motion_client.cpp
      motion_client/robosub_motion_client.cpp
)
