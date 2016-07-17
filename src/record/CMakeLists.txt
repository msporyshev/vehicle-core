cmake_minimum_required(VERSION 2.8.3)
project(record)

find_package(Boost REQUIRED COMPONENTS regex system program_options)

auv_pkg(
    INCLUDE_DIRS
    AUVDEP libauv libipc
    ROSDEP rosbag
    MSGDIR msg
    MSGDEP std_msgs
)

add_node(record_master src/record_master.cpp)
add_node(record src/record.cpp)

target_link_libraries(record
    ${Boost_LIBRARIES}
)

target_link_libraries(record_master
    ${Boost_LIBRARIES}
)