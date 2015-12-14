cmake_minimum_required(VERSION 2.8.3)
project(libipc)


auv_pkg(
    INCLUDE_DIRS ..
    LIBRARIES libipc
    MSGDEP std_msgs
    MSGDIR msg
)

add_lib(libipc
    ipc.cpp
)

add_node(pub pub.cpp)
add_node(sub_async sub_async.cpp)
add_node(sub_sync sub_sync.cpp)

target_link_libraries(pub
    libipc
)

target_link_libraries(sub_async
    libipc
)

target_link_libraries(sub_sync
    libipc
)

