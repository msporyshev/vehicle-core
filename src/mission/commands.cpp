#include "commands.h"

#include <vision/CmdSwitchCamera.h>
#include <supervisor/CmdDeviceKey.h>
#include <dsp/CmdSendCommand.h>

#include <chrono>
#include <thread>

using std::chrono::milliseconds;

namespace { // namespace
    void switch_device_state(SupervisorDevices device, int state, ros::Publisher& pub)
    {
        supervisor::CmdDeviceKey msg;
        msg.id = static_cast<unsigned char>(device);
        msg.state = state;

        pub.publish(msg);
    }

    void switch_device(SupervisorDevices device, int delay_ms, ros::Publisher& pub)
    {
        switch_device_state(device, 1, pub);
        std::this_thread::sleep_for(milliseconds(delay_ms));
        switch_device_state(device, 0, pub);
    }

    void dsp_send_command(dsp::CommandType command, ros::Publisher& pub)
    {
        dsp::CmdSendCommand msg;
        msg.command = static_cast<unsigned char>(command);
        pub.publish(msg);
    }
} // namespace

void Commands::set_recognizers(Camera camera_type,
        std::vector<std::string> recognizers = std::vector<std::string>())
{
    vision::CmdSwitchCamera msg;
    msg.camera_type = static_cast<unsigned char>(camera_type);
    msg.recognizers = recognizers;
    switch_camera_pub_.publish(msg);
}


void Commands::switch_off_vision()
{
    set_recognizers(Camera::None);
}

void Commands::set_dsp_mode(dsp::CommandType mode)
{
    for (int i = 0; i < 3; i++) {
        dsp_send_command(dsp::CommandType::DspOff, switch_pinger_pub_);
        std::this_thread::sleep_for(milliseconds(300));
    }
    dsp_send_command(mode, switch_pinger_pub_);
    std::this_thread::sleep_for(milliseconds(300));
    dsp_send_command(dsp::CommandType::DspOn, switch_pinger_pub_);
}

void Commands::drop_cargo(int delay)
{
    switch_device(cargo_, delay, switch_device_pub_);
    cargo_ = SupervisorDevices::Cargo_2;
}

