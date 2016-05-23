cmake_minimum_required(VERSION 2.8.3)
project(mission)

auv_pkg(
    INCLUDE_DIRS ..
    AUVDEP libipc motion video
    MSGDIR msg
    MSGDEP std_msgs motion video supervisor libauv dsp
)

add_node(mission
    main.cpp
    mission.cpp
    sample_task.cpp
    gate_task.cpp
    camera_model.cpp
    commands.cpp
    test_pinger_task.cpp
    flare_task.cpp
    flare_vision_task.cpp
    drum_task.cpp
    )
