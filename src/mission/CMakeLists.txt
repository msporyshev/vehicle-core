cmake_minimum_required(VERSION 2.8.3)
project(mission)


auv_pkg(
    INCLUDE_DIRS ..
    AUVDEP libipc motion video
    MSGDEP motion video supervisor
)

add_node(mission mission.cpp sample_task.cpp camera_model.cpp commands.cpp)
