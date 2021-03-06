cmake_minimum_required(VERSION 2.8.3)
project(libauv)

include(FindPkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

find_package(Boost REQUIRED COMPONENTS filesystem)

auv_pkg(
    INCLUDE_DIRS include . config_reader utils
    LIBRARIES libauv
    AUVDEP libipc
    ROSDEP roslib roscpp
    MSGDEP std_msgs
)

add_lib(libauv
    config_reader/yaml_reader.cpp
    config_reader/config_reader.cpp
    utils/basic.cpp
    utils/earth.cpp
    utils/math_u.cpp
)

target_link_libraries(libauv
    ${Boost_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)
