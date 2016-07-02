cmake_minimum_required(VERSION 2.8.3)
project(tcu)

auv_pkg(
    INCLUDE_DIRS ..
    AUVDEP libipc
    MSGDIR msg
)

add_node(motion_node tcu.cpp)

file(GLOB_RECURSE CONFIGS "tcu.yml")
install(FILES ${CONFIGS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/config)
