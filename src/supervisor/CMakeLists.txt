cmake_minimum_required(VERSION 2.8.3)
project(supervisor)

find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(
	cfg/SupervisorCalibration.cfg
    cfg/SupervisorEthernet.cfg
    cfg/SupervisorDevices.cfg
    cfg/SupervisorPeriods.cfg
)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc dynamic_reconfigure
    MSGDIR msg
    MSGDEP std_msgs
)



file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(supervisor_node ${SOURCES})

add_deps(supervisor_node)
add_dependencies(supervisor_node ${PROJECT_NAME}_gencfg)

target_link_libraries(supervisor_node
    ${Boost_LIBRARIES}
)