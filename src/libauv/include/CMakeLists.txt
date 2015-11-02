cmake_minimum_required(VERSION 2.8.3)

project(auv)

find_package(Boost REQUIRED system thread)

file(GLOB_RECURSE SOURCES "*.cpp")

set(INCLUDES "")
foreach(file_path ${SOURCES})
    get_filename_component(dir_path ${file_path} PATH)

    list(APPEND INCLUDES ${dir_path})
endforeach()
list(REMOVE_DUPLICATES INCLUDES)

set(IPC_GEN_EXECUTABLE ${CMAKE_BINARY_DIR}/utils/ipc_msg_generator/ipc_msg_generator)

set(MSG_SRCS "")
set(MSG_HDRS "")

file(GLOB_RECURSE MSGS "*.msg")
foreach(MSG_FILE ${MSGS})
    get_filename_component(MSG_PATH ${MSG_FILE} PATH)
    get_filename_component(ABS_FILE ${MSG_FILE} ABSOLUTE)
    get_filename_component(FILE_WE ${MSG_FILE} NAME_WE)

    set(CPP_FILE "${MSG_PATH}/${FILE_WE}.cpp")
    set(H_FILE "${MSG_PATH}/${FILE_WE}.h")

    list(APPEND MSG_SRCS ${CPP_FILE})
    list(APPEND MSG_HDRS ${H_FILE})

    add_custom_command(
        OUTPUT ${CPP_FILE} ${H_FILE}
        COMMAND ${IPC_GEN_EXECUTABLE} ${MSG_FILE}
        DEPENDS ${ABS_FILE}
        COMMENT "Running ipc_msg_generator on ${MSG_FILE}"
    )
endforeach()

set_source_files_properties(${MSG_SRCS} ${MSG_HDRS} PROPERTIES GENERATED TRUE)

include_directories(./ ${INCLUDES} ${Boost_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} ${SOURCES} ${MSG_SRCS})

add_dependencies(${PROJECT_NAME} ipc_msg_generator)

target_link_libraries(auv
    ipc
    opencv_core
    ${YamlCpp_LIBRARIES}
    ${Boost_LIBRARIES}
)
