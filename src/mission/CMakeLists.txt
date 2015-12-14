cmake_minimum_required(VERSION 2.8.3)
project(mission)


auv_pkg(
    INCLUDE_DIRS ..
    AUVDEP libipc motion video
    MSGDEP motion video
)

add_node(mission mission.cpp)
