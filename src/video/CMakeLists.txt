cmake_minimum_required(VERSION 2.8.3)
project(video)

find_package(OpenCV COMPONENTS)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
)

auv_pkg(
    AUVDEP libauv libipc
    MSGDEP camera
    MSGDIR msg
)

add_node(video_node main.cpp image_algorithm.cpp)


target_link_libraries(video_node
  ${OpenCV_LIBRARIES}
)

