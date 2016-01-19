cmake_minimum_required(VERSION 2.8.3)
project(motion)

auv_pkg(
    INCLUDE_DIRS include motion_client
    LIBRARIES motion_client
    AUVDEP libipc libauv
    MSGDEP std_msgs
    MSGDIR msg
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(motion_node ${SOURCES})

file(GLOB_RECURSE SOURCES "motion_client/*.cpp")
add_lib(motion_client
      ${SOURCES}
)

file(GLOB_RECURSE CONFIGS "motion.yml")
install(FILES ${CONFIGS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/config)
