cmake_minimum_required(VERSION 2.8.3)
project(motion)

auv_pkg(
    INCLUDE_DIRS include motion_client
    LIBRARIES motion_client
    AUVDEP libipc libauv navig tcu
    MSGDEP std_msgs navig tcu
    MSGDIR msg
)

add_node(motion_node
    src/main.cpp
    src/motion_server.cpp
    src/registry.cpp
    src/regul_storage.cpp
    src/regulators/depth_regulator.cpp
    src/regulators/heading_regulator.cpp
    src/regulators/height_regulator.cpp
    src/regulators/pitch_regulator.cpp
    src/regulators/position_regulator.cpp
    src/regulators/regulator.cpp
    src/regulators/speedy_vert_regulator.cpp
    src/regulators/target_distance_regulator.cpp
    src/regulators/target_regulator.cpp
    src/regulators/thrust_regulator.cpp
    src/regulators/velocity_regulator.cpp
    src/controllers/pid_controller.cpp
)

add_lib(motion_client
    motion_client/motion_client.cpp
    motion_client/robosub_motion_client.cpp
    motion_client/robosub_conf_motion_client.cpp
)

file(GLOB_RECURSE CONFIGS "motion.yml")
install(FILES ${CONFIGS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/config)
