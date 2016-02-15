#include "commands.h"

#include <video/CmdSwitchCamera.h>
#include <supervisor/CmdDeviceKey.h>

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
        switch_device(device, 1, pub);
        std::this_thread::sleep_for(milliseconds(delay_ms));
        switch_device(device, 0, pub);
    }
} // namespace

void Commands::set_recognizers(Camera camera_type,
        std::vector<std::string> recognizers = std::vector<std::string>())
{
    video::CmdSwitchCamera msg;
    msg.camera_type = static_cast<unsigned char>(camera_type);
    msg.recognizers = recognizers;
    switch_camera_pub_.publish(msg);
}


void Commands::switch_off_vision()
{
    set_recognizers(Camera::None);
}

void Commands::drop_cargo(int delay)
{
    switch_device(cargo_, delay, switch_device_pub_);
    cargo_ = SupervisorDevices::Cargo_2;
}

