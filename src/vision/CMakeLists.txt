cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(OpenCV COMPONENTS)
find_package(Boost REQUIRED program_options filesystem)

include_directories(
  include/vision/
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

auv_pkg(
  INCLUDE_DIRS include
  AUVDEP libauv libipc navig
  MSGDEP libauv std_msgs navig
  ROSDEP cv_bridge image_transport
  MSGDIR msg
)

add_node(vision_node
  main.cpp
  image_algorithm.cpp
  bin.cpp
  channel.cpp
  stripe.cpp
  gate.cpp
  fargate.cpp
  circle.cpp
  )

add_node(hsv_visualizer hsv_visualizer.cpp image_algorithm)

target_link_libraries(hsv_visualizer
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(vision_node
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

