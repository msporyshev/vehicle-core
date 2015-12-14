MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})
    SET(larg_names ${arg_names})
    LIST(FIND larg_names "${arg}" is_arg_name)
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})
      LIST(FIND loption_names "${arg}" is_option)
      IF (is_option GREATER -1)
         SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
         SET(current_arg_list ${current_arg_list} ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

function(auv_pkg)
  set(multiValueArgs INCLUDE_DIRS LIBRARIES ROSDEP AUVDEP MSGDEP MSGDIR FILES)
  # set(singleValueArgs MSGDIR)

  PARSE_ARGUMENTS(AUV_PKG "${multiValueArgs}" "" "" ${ARGN} )

  find_package(catkin REQUIRED COMPONENTS roscpp message_generation ${AUV_PKG_MSGDEP} ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP} )
  message("RAW LIBS ${catkin_LIBRARIES}")


  if(AUV_PKG_MSGDIR)
    # get_filename_component(MYDIR "/Uasdf/asdf/../filename.h" DIRECTORY)
    set(MSGDIR ${PROJECT_SOURCE_DIR}/${AUV_PKG_MSGDIR})
    # message("MYDIR: ${MYDIR}")
    file(GLOB_RECURSE MSG_SOURCES "${AUV_PKG_MSGDIR}/*.msg")
    set(msg_filenames)
    foreach(source ${MSG_SOURCES})
      get_filename_component(filename ${source} NAME)
      list(APPEND msg_filenames ${filename})
    endforeach()

    message("WHAAAT: ${msg_filenames}")
    add_message_files(DIRECTORY ${MSGDIR} FILES ${msg_filenames})
    set(${PROJECT_NAME}_MESSAGE_FILES ${${PROJECT_NAME}_MESSAGE_FILES} PARENT_SCOPE)
  endif()


  set(CATKIN_DEPS ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP})

  if(AUV_PKG_MSGDIR OR AUV_PKG_MSGDEP)
    # message("WHAAAT: ${AUV_PKG_FILES}")
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

  set(catkin_LIBS ${catkin_LIBRARIES} PARENT_SCOPE)
  set(catkin_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} PARENT_SCOPE)
endfunction()

function(add_deps node_name)
  set(MSGDEPS ${catkin_EXPORTED_TARGETS})

  if(${PROJECT_NAME}_MESSAGE_FILES)
    list(APPEND MSGDEPS ${PROJECT_NAME}_generate_messages_cpp)
  endif()

  if (MSGDEPS)
    add_dependencies(${node_name} ${MSGDEPS})
  endif()

  target_link_libraries(${node_name} ${catkin_LIBS})
endfunction()

function(add_node node_name)
  add_executable(${node_name} ${ARGN})
  add_deps(${node_name})
endfunction()

function(add_lib lib_name)
  add_library(${lib_name} ${ARGN})
  add_deps(${lib_name})
endfunction()