cmake_minimum_required(VERSION 2.8.3)
project(dsp)

auv_pkg(
    INCLUDE_DIRS ..
    AUVDEP libipc navig 
    MSGDEP std_msgs
    MSGDIR msg
)

add_node(dsp dsp.cpp connector.cpp)

target_link_libraries(dsp
    -lftd2xx    
)
