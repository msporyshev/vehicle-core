cmake_minimum_required(VERSION 2.8.3)
project(libipc)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs genmsg)


add_message_files(DIRECTORY msg FILES MsgTestMsg.msg)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES libipc
    CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ..
)

## Declare a C++ library
add_library(libipc
    ipc.cpp
)
target_link_libraries(libipc
    ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(libipc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(libipc_node src/libipc_node.cpp)
add_executable(pub pub.cpp)
add_executable(sub_async sub_async.cpp)
add_executable(sub_sync sub_sync.cpp)

target_link_libraries(pub
    ${catkin_LIBRARIES}
    libipc
)

target_link_libraries(sub_async
    ${catkin_LIBRARIES}
    libipc
)

target_link_libraries(sub_sync
    ${catkin_LIBRARIES}
    libipc
)

