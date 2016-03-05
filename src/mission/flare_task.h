#pragma once

#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>

namespace {
enum class State
{
    Initialization,
    ListenToFirstPing,
    GoToFlare,
    BumpFlare,
    Finalize,
    Terminal
};
}

enum class Zone
{
    CloseIn,
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
    AUTOPARAM(double, timeout_listen_first_ping_);
    AUTOPARAM(double, timeout_go_flare_);
    AUTOPARAM(double, timeout_bump_flare_);
    AUTOPARAM(double, timeout_finalize_);
    AUTOPARAM(double, timeout_regul_);
    AUTOPARAM(double, timeout_lose_pinger_);
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, thrust_close_in_);
    AUTOPARAM(double, thrust_finalize_);
    AUTOPARAM(double, close_in_border_);
    AUTOPARAM(double, infinity_);
    AUTOPARAM(int, pings_needed_);
    AUTOPARAM(int, filter_size_);
    AUTOPARAM(bool, use_median_);
    AUTOPARAM(int, pinger_id_);

    Zone cur_zone_;
    std::vector<ZoneInfo> zones_;
    double cur_heading_;
    double cur_dist_;
    double timestamp_;
    std::vector<double> pinger_headings_;
    ipc::Subscriber<dsp::MsgBeacon> sub_ping_;
    bool ping_found_ = false;
    int count_ = 0;

    void init_ipc(ipc::Communicator& com);
    void init_zones();
    Zone update_zone(const dsp::MsgBeacon& msg);
};