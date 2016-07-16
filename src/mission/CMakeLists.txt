cmake_minimum_required(VERSION 2.8.3)
project(mission)

auv_pkg(
    INCLUDE_DIRS . camera_model
    AUVDEP navig libipc motion vision
    MSGDIR msg
    MSGDEP navig std_msgs motion vision supervisor libauv dsp
)

add_node(mission
    main.cpp
    mission.cpp
    commands.cpp
    camera_model/camera_model.cpp

    test_pinger_task.cpp
    gate_task.cpp
    orange_lane_task.cpp
    navigate_channel_task.cpp
    channel_spin_task.cpp
)

add_node(visual_nav
    camera_model/camera_model.cpp
    visual_nav_node/main.cpp
)
