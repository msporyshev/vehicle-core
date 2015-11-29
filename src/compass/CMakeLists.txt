cmake_minimum_required(VERSION 2.8.3)
project(compass)

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation roscpp libauv libipc)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options)

# include(FindPkgConfig)
# pkg_check_modules(YamlCpp yaml-cpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES msgCompassAcceleration.msg
                                      msgCompassAngle.msg
                                      msgCompassAngleRate.msg
                                      msgCompassMagnetometer.msg
                                      msgCompassQuaternion.msg)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS std_msgs roscpp libauv libipc
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
)
link_directories(${YamlCpp_LIBRARY_DIRS})

## Declare a C++ executable
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(compass_node ${SOURCES})

## Specify libraries to link a library or executable target against

target_link_libraries(compass_node
  	${catkin_LIBRARIES}
  	-lusb-1.0
  	${YamlCpp_LIBRARIES}
  	${Boost_LIBRARIES}
)