#pragma once

#include <libipc/ipc.h>
#include <video/common.h>

#include <vector>
#include <string>

#include <supervisor/supervisor_devices.h>
#include <video/CmdSwitchCamera.h>
#include <supervisor/CmdDeviceKey.h>

class Commands
{
public:
    Commands(ipc::Communicator& comm)
            : switch_camera_pub_(comm.advertise_cmd<video::CmdSwitchCamera>("video"))
            , switch_device_pub_(comm.advertise_cmd<supervisor::CmdDeviceKey>("supervisor"))
    {}

    void set_recognizers(Camera camera_type, std::vector<std::string> recognizers);
    void switch_off_vision();

    void drop_cargo(int delay = 100);
private:
    ros::Publisher switch_camera_pub_;
    ros::Publisher switch_device_pub_;

    SupervisorDevices cargo_ = SupervisorDevices::Cargo_1;
};