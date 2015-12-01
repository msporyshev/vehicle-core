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
  set(multiValueArgs INCLUDE_DIRS LIBRARIES ROSDEP AUVDEP MSGDEP)

  PARSE_ARGUMENTS(AUV_PKG "${multiValueArgs}" "" "" ${ARGN} )

  find_package(catkin REQUIRED COMPONENTS roscpp message_generation ${AUV_MSGDEP} ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP} )

  generate_messages(DEPENDENCIES ${AUV_PKG_MSGDEP})

  catkin_package(
    INCLUDE_DIRS ${AUV_PKG_INCLUDE_DIRS}
    LIBRARIES ${AUV_PKG_LIBRARIES}
    CATKIN_DEPENDS ${AUV_PKG_ROSDEP} ${AUV_PKG_AUVDEP}
    )


  include_directories(${AUV_PKG_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
  link_directories(
    ${catkin_LIBRARY_DIRS}
  )

  set(catkin_LIBS ${catkin_LIBRARIES} PARENT_SCOPE)


endfunction()

function(add_node node_name)
  add_executable(${node_name} ${ARGN})
  target_link_libraries(${node_name} ${catkin_LIBS})
  add_dependencies(${node_name} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
endfunction()

function(add_lib lib_name)
  add_library(${lib_name} ${ARGN})
  target_link_libraries(${lib_name} ${catkin_LIBS})
endfunction()