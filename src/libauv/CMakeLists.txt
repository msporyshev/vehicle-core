cmake_minimum_required(VERSION 2.8.3)
project(libauv)

include(FindPkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

find_package(Boost REQUIRED COMPONENTS filesystem)

auv_pkg(
    INCLUDE_DIRS include . camera_model config_reader utils point
    LIBRARIES libauv
    AUVDEP libipc
    ROSDEP roslib roscpp
    MSGDEP std_msgs
    MSGDIR msg
)

add_lib(libauv
    config_reader/yaml_reader.cpp
    config_reader/config_reader.cpp
    camera_model/camera_model.cpp
    utils/basic.cpp
    utils/earth.cpp
    utils/math_u.cpp
    point/point.cpp
)

target_link_libraries(libauv
    ${Boost_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)
