#pragma once

#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>

enum class State
{
    Initialization,
    ListenToFirstPing,
    GoToFlare,
    BumpFlare,
    Finalize,
    Terminal
};

enum class Zone
{
    Far,
    Middle,
    Near,
    Bump
};

struct ZoneInfo
{
    Zone zone;
    double min;
    double max;
    int pings_in_row = 0;

    ZoneInfo() {}
    ZoneInfo(Zone zone, double min, double max)
            : zone(zone)
            , min(min)
            , max(max)
    {}
};

class FlareTask: public Task<State>
{
public:
    FlareTask(const YamlReader& cfg, ipc::Communicator& com);

    State handle_initialization();
    State handle_listen_first_ping();
    State handle_go_flare();
    State handle_bump_flare();
    State handle_finalize();

    void handle_pinger_found(const dsp::MsgBeacon& msg);

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_listen_firt_ping_);
    AUTOPARAM(double, timeout_go_flare_);
    AUTOPARAM(double, timeout_bump_flare_);
    AUTOPARAM(double, timeout_finalize_);
    AUTOPARAM(double, timeout_regul_);
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, thrust_far_);
    AUTOPARAM(double, thrust_middle_);
    AUTOPARAM(double, thrust_finalize_);
    AUTOPARAM(double, far_border_);
    AUTOPARAM(double, infinity_);
    AUTOPARAM(double, middle_border_);
    AUTOPARAM(double, close_border_);
    AUTOPARAM(int, pings_needed_);
    AUTOPARAM(double, close_koef_);
    AUTOPARAM(double, max_close_thrust_);

    Zone cur_zone_;
    std::vector<ZoneInfo> zones_;
    dsp::MsgBeacon pinger_state_;
    ipc::Subscriber<dsp::MsgBeacon> sub_ping_;
    bool ping_found_ = false;

    void init_ipc(ipc::Communicator& com);
    void init_zones();
    Zone update_zone(const dsp::MsgBeacon& msg);
    libauv::Point2d calc_thrust(double cur_heading, double pinger_heading);
    void limit_max_thrust(double& thrust, double max_value);
};