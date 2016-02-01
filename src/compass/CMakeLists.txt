cmake_minimum_required(VERSION 2.8.3)
project(compass)

find_package(Boost REQUIRED COMPONENTS system program_options)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
    MSGDIR msg
    MSGDEP std_msgs
)

file(GLOB_RECURSE SOURCES "src/inemo/*.cpp")
add_node(inemo_node ${SOURCES})

add_node(x_sens_node src/x-sens/compass.cpp src/x-sens/main.cpp src/x-sens/MTI.cpp)

# file(GLOB_RECURSE SOURCES "src/x-sens/calibration/mti_compass_calibration_data_collect.cpp")
# add_node(x_sens_calib_collect_node ${SOURCES})

# file(GLOB_RECURSE SOURCES "src/x-sens/calibration/mti_compass_calibration_data_store.cpp")
# add_node(x_sens_calib_store_node ${SOURCES})

target_link_libraries(inemo_node
	-lusb-1.0
    ${Boost_LIBRARIES}
)

target_link_libraries(x_sens_node
    ${Boost_LIBRARIES}
)
