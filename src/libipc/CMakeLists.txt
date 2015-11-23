cmake_minimum_required(VERSION 2.8.3)
project(libipc)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS libauv roscpp std_msgs genmsg)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
 # LIBRARIES libipc
#  CATKIN_DEPENDS roscpp
    DEPENDS libauv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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
add_executable(main main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(libipc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(main
    ${catkin_LIBRARIES}
    libipc
)
