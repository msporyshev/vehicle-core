cmake_minimum_required(VERSION 2.8.3)
project(tcu)

auv_pkg(
    AUVDEP libipc supervisor
    MSGDIR msg
)

add_node(tcu tcu.cpp tcu_surface.cpp main.cpp)

file(GLOB_RECURSE CONFIGS "tcu.yml")
install(FILES ${CONFIGS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/config)
