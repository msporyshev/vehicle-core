cmake_minimum_required(VERSION 2.8.3)
project(video)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS libauv)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES video
    CATKIN_DEPENDS libauv
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(video_node main.cpp)

# add_dependencies(video_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(video_node
    ${catkin_LIBRARIES}
)
