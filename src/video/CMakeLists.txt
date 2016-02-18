cmake_minimum_required(VERSION 2.8.3)
project(video)

find_package(OpenCV COMPONENTS)
find_package(Boost REQUIRED program_options filesystem)

include_directories(
  include/video/
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

auv_pkg(
  INCLUDE_DIRS include
  AUVDEP libauv libipc
  MSGDEP libauv std_msgs
  ROSDEP cv_bridge
  MSGDIR msg
)

add_node(video_node main.cpp image_algorithm.cpp stripe.cpp gate.cpp)

target_link_libraries(video_node
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

