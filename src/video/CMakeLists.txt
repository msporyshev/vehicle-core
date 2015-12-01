cmake_minimum_required(VERSION 2.8.3)
project(video)


auv_pkg(
    AUVDEP libipc
)

add_node(video_node main.cpp)

