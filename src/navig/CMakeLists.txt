cmake_minimum_required(VERSION 2.8.3)
project(navig)

find_package(catkin REQUIRED COMPONENTS libipc dvl gps supervisor roscpp message_generation)

add_message_files(
    FILES
    MsgNavigAccelerations.msg
    MsgNavigAngles.msg
    MsgNavigDepth.msg
    MsgNavigHeight.msg
    MsgNavigPosition.msg
    MsgNavigRates.msg
    MsgNavigVelocity.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES libipc
    CATKIN_DEPENDS message_runtime libipc gps dvl supervisor
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(navig navig.cpp main.cpp)

add_dependencies(navig ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


target_link_libraries(navig
    libipc
    ${catkin_LIBRARIES}
)
