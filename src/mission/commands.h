#pragma once

#include <libipc/ipc.h>
#include <video/common.h>

#include <vector>
#include <string>

#include <supervisor/supervisor_devices.h>
#include <video/CmdSwitchCamera.h>
#include <supervisor/CmdDeviceKey.h>
#include <dsp/CmdSendCommand.h>

class Commands
{
public:
    Commands(ipc::Communicator& comm)
            : switch_camera_pub_(comm.advertise_cmd<video::CmdSwitchCamera>("video"))
            , switch_device_pub_(comm.advertise_cmd<supervisor::CmdDeviceKey>("supervisor"))
            , switch_pinger_pub_(comm.advertise_cmd<dsp::CmdSendCommand>("dsp"))
    {}

    void set_recognizers(Camera camera_type, std::vector<std::string> recognizers);
    void switch_off_vision();

    void set_dsp_freq_37500();
    void set_dsp_freq_20000();
    void set_dsp_state(bool new_state);

    void drop_cargo(int delay = 100);
private:
    ros::Publisher switch_camera_pub_;
    ros::Publisher switch_device_pub_;
    ros::Publisher switch_pinger_pub_;

    SupervisorDevices cargo_ = SupervisorDevices::Cargo_1;
};