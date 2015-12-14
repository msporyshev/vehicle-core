cmake_minimum_required(VERSION 2.8.3)
project(tcu)

find_package(catkin REQUIRED COMPONENTS libipc supervisor motion roscpp message_generation)

# add_message_files(
    # FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
    INCLUDE_DIRS ..
    LIBRARIES libipc
    CATKIN_DEPENDS message_runtime libipc
)

set(SOURCE_DIR ".")

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB SOURCES "${SOURCE_DIR}/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}
    libipc
    ${catkin_LIBRARIES}
)