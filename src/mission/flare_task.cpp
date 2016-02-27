#include "flare_task.h"

#include <algorithm>

#include <libauv/utils/math_u.h>

#include <ostream>
#include <algorithm>

double get_median(std::vector<double> values)
{
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
}

FlareTask::FlareTask(const YamlReader& cfg, ipc::Communicator& com): Task<State>(cfg, com, State::Initialization)
    ,pinger_headings_(filter_size_.get())
{
    state_machine_.REG_STATE(State::Initialization, handle_initialization, timeout_initialization_.get(), State::ListenToFirstPing);
    state_machine_.REG_STATE(State::ListenToFirstPing, handle_listen_first_ping, timeout_listen_first_ping_.get(), State::BumpFlare);
    state_machine_.REG_STATE(State::GoToFlare, handle_go_flare, timeout_go_flare_.get(), State::BumpFlare);
    state_machine_.REG_STATE(State::BumpFlare, handle_bump_flare, timeout_bump_flare_.get(), State::Finalize);
    state_machine_.REG_STATE(State::Finalize, handle_finalize, timeout_finalize_.get(), State::Terminal);

    init_ipc(com);
    init_zones();
}

State FlareTask::handle_initialization()
{
    motion_.fix_pitch();
    motion_.fix_depth(start_depth_.get());
    ROS_INFO_STREAM("Initialization has been completed. Working on depth: " << navig_.last_depth());
    
    cmd_.set_dsp_mode(dsp::CommandType::Freq37500);
    ping_found_ = false;
    count_ = 0;

    return State::ListenToFirstPing;
}

State FlareTask::handle_listen_first_ping()
{
    if (ping_found_) {
        ROS_INFO_STREAM("Enough pings were heard");
        return State::GoToFlare;
    }

    double last_head = navig_.last_head();
    motion_.fix_heading(normalize_degree_angle(last_head + heading_delta_.get()), WaitMode::DONT_WAIT);
    ROS_INFO_STREAM("Pinger hasn't ever been heard. Heading was changed from " << last_head << " to " << navig_.last_head());
    return State::ListenToFirstPing;
}

State FlareTask::handle_go_flare()
{
    if (!ping_found_) {
        return State::GoToFlare;
    }
    ping_found_ = false;

    ROS_INFO_STREAM("Heading to pinger: " << cur_heading_);
    motion_.fix_heading(cur_heading_, WaitMode::DONT_WAIT);

    if (cur_zone_ == Zone::Bump) {
        ROS_INFO_STREAM("Working in bump zone");
        return State::BumpFlare;
    }

    motion_.thrust_forward(thrust_close_in_.get(), timeout_regul_.get(), WaitMode::DONT_WAIT);
    ROS_INFO_STREAM("Working in CloseIn zone");
    ROS_INFO_STREAM("\tCurrent pinger heading: " << cur_heading_);
    ROS_INFO_STREAM("\tCurrent thrust: " << thrust_close_in_.get());

    return State::GoToFlare;
}

State FlareTask::handle_bump_flare()
{
    motion_.turn_right(179);
    motion_.turn_right(179);
    return State::Finalize;
}

State FlareTask::handle_finalize()
{
    motion_.fix_heading(navig_.last_head());
    motion_.thrust_backward(thrust_finalize_.get(), timeout_finalize_.get());
    ROS_INFO_STREAM("Finalize stage has been completed with heading: " << navig_.last_head()
        << " and thrust " << thrust_finalize_.get());
    return State::Terminal;
}

void FlareTask::handle_pinger_found(const dsp::MsgBeacon& msg)
{
    if (msg.beacon_type != 0) {
        ROS_INFO_STREAM("Signal from incorrect pinger. Ignore.");
        return;
    }

    pinger_headings_[count_++ % filter_size_.get()] = msg.heading;

    cur_heading_ = use_median_.get() ? get_median(pinger_headings_) : msg.heading;
    cur_dist_ = msg.distance;

    ROS_INFO_STREAM("Current distance to pinger: " << msg.distance);
    cur_zone_ = update_zone(msg);
    ping_found_ = true;
}

void FlareTask::init_ipc(ipc::Communicator& com)
{
    sub_ping_ = com.subscribe("dsp", &FlareTask::handle_pinger_found, this);
}

void FlareTask::init_zones()
{
    zones_.emplace_back(Zone::CloseIn, close_in_border_.get(), infinity_.get());
    zones_.emplace_back(Zone::Bump, 0, close_in_border_.get());
}

Zone FlareTask::update_zone(const dsp::MsgBeacon& msg)
{
    Zone zone;
    for (auto& z : zones_) {
        if (cur_dist_ < z.max && cur_dist_ >= z.min) {
            ++z.pings_in_row;
            if (z.pings_in_row >= pings_needed_.get() &&
                zone != z.zone) {
                zone = z.zone;
            }
        } else {
            z.pings_in_row = 0;
        }
    }
    return zone;
}

REGISTER_TASK(FlareTask, flare_task);
