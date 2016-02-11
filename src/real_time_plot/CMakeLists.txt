cmake_minimum_required(VERSION 2.8.3)
project(real_time_plot)

find_package(catkin REQUIRED COMPONENTS roscpp libipc navig message_generation)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

generate_messages(DEPENDENCIES navig)

catkin_package(
  CATKIN_DEPENDS roscpp motion
  LIBRARIES libipc navig motion
)

set(SOURCES ${PROJECT_SOURCE_DIR}/mainwindow.cpp ${PROJECT_SOURCE_DIR}/qcustomplot.cpp)
set(HEADERS ${PROJECT_SOURCE_DIR}/mainwindow.h ${PROJECT_SOURCE_DIR}/qcustomplot.h)
set(UIS ${PROJECT_SOURCE_DIR}/mainwindow.ui)
set(BINARIES ${PROJECT_SOURCE_DIR}/main.cpp)

qt5_wrap_cpp(QT_MOC_HPP ${HEADERS})
qt5_wrap_ui(QT_FORMS_HPP ${UIS})

include_directories(${PROJECT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories (${PROJECT_BINARY_DIR})

add_library(LIB SHARED ${SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})

target_link_libraries(LIB ${QT_LIBRARIES}
                      ${catkin_LIBRARIES} Qt5::Widgets Qt5::PrintSupport
                      motion_client)

add_executable(real_time_plot ${BINARIES})
target_link_libraries(real_time_plot LIB)

add_dependencies(real_time_plot ${catkin_EXPORTED_TARGETS})

install(TARGETS real_time_plot
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
