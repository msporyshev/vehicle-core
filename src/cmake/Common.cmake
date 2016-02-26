macro(auv_pkg)
  set(multiValueArgs INCLUDE_DIRS LIBRARIES ROSDEP AUVDEP MSGDEP MSGDIR FILES)
  cmake_parse_arguments(AUV_PKG "" "" "${multiValueArgs}" ${ARGN})

  find_package(catkin REQUIRED COMPONENTS roscpp message_generation ${AUV_PKG_MSGDEP} ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP} )


  if(AUV_PKG_MSGDIR)
    set(MSGDIR ${PROJECT_SOURCE_DIR}/${AUV_PKG_MSGDIR})
    file(GLOB_RECURSE MSG_SOURCES "${AUV_PKG_MSGDIR}/*.msg")
    set(msg_filenames)
    foreach(source ${MSG_SOURCES})
      get_filename_component(filename ${source} NAME)
      list(APPEND msg_filenames ${filename})
    endforeach()

    add_message_files(DIRECTORY ${MSGDIR} FILES ${msg_filenames})
    set(${PROJECT_NAME}_MESSAGE_FILES ${${PROJECT_NAME}_MESSAGE_FILES} PARENT_SCOPE)
  endif()


  set(CATKIN_DEPS ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP})

  if(AUV_PKG_MSGDIR OR AUV_PKG_MSGDEP)
    generate_messages(DEPENDENCIES ${AUV_PKG_MSGDEP})
    list(APPEND CATKIN_DEPS message_runtime)
  endif()


  catkin_package(
    INCLUDE_DIRS ${AUV_PKG_INCLUDE_DIRS}
    LIBRARIES ${AUV_PKG_LIBRARIES}
    CATKIN_DEPENDS ${CATKIN_DEPS}
    )


  include_directories(${AUV_PKG_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  link_directories(
    ${catkin_LIBRARY_DIRS}
  )
endmacro()

macro(add_deps node_name)
  set(MSGDEPS ${catkin_EXPORTED_TARGETS})

  if(${PROJECT_NAME}_MESSAGE_FILES)
    list(APPEND MSGDEPS ${PROJECT_NAME}_generate_messages_cpp)
  endif()

  if (MSGDEPS)
    add_dependencies(${node_name} ${MSGDEPS})
  endif()

  target_link_libraries(${node_name} ${catkin_LIBRARIES})
endmacro()

macro(add_node node_name)
  add_executable(${node_name} ${ARGN})
  add_deps(${node_name})
endmacro()

macro(add_lib lib_name)
  add_library(${lib_name} ${ARGN})
  add_deps(${lib_name})
endmacro()