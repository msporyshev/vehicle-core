cmake_minimum_required(VERSION 2.8.3)
project(supervisor)

find_package(Boost REQUIRED COMPONENTS system program_options)

add_message_files(DIRECTORY msg FILES   MsgSupervisorAdc.msg
                                        MsgSupervisorCompensator.msg
                                        MsgSupervisorDepth.msg
                                        MsgSupervisorExternalAdc.msg
                                        MsgSupervisorLeak.msg
                                        MsgSupervisorShortCircuit.msg
                                        MsgSupervisorDevicesStatus.msg
                                        CmdSupervisorCan.msg
                                        CmdSupervisorCas.msg
                                        CmdSupervisorConfigureUdp.msg
                                        CmdSupervisorDevicesKeys.msg
                                        CmdSupervisorFirmware.msg
                                        CmdSupervisorPwm.msg
                                        CmdSupervisorSystemFlags.msg
                                        CmdSupervisorUart.msg)

auv_pkg(
    INCLUDE_DIRS include
    AUVDEP libauv libipc
)

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_node(supervisor_node ${SOURCES})

target_link_libraries(supervisor_node
    ${Boost_LIBRARIES}
)