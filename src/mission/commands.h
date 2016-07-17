#pragma once

#include <libipc/ipc.h>
#include <vision/common.h>

#include <vector>
#include <string>

#include <supervisor/supervisor_devices.h>
#include <vision/CmdSwitchCamera.h>
#include <supervisor/CmdDeviceKey.h>
#include <dsp/CmdSendCommand.h>
#include <dsp/commands.h>

class Commands
{
public:
    Commands(ipc::Communicator& comm)
            : switch_camera_pub_(comm.advertise_cmd<vision::CmdSwitchCamera>("vision"))
            , switch_device_pub_(comm.advertise_cmd<supervisor::CmdDeviceKey>("supervisor"))
            , switch_pinger_pub_(comm.advertise_cmd<dsp::CmdSendCommand>("dsp"))
    {}

    void set_recognizers(Camera camera_type, std::vector<std::string> recognizers);
    void switch_off_vision();

    void set_dsp_mode(dsp::CommandType mode);

    void grab();
    void ungrab();
    void drop_cargo(int delay = 100);
private:
    ros::Publisher switch_camera_pub_;
    ros::Publisher switch_device_pub_;
    ros::Publisher switch_pinger_pub_;

    SupervisorDevices cargo_ = SupervisorDevices::Cargo_1;
};