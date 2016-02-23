#include "flare_task.h"

#include <libauv/utils/math_u.h>

#include <ostream>

FlareTask::FlareTask(const YamlReader& cfg, ipc::Communicator& com): Task<State>(cfg, com, State::Initialization)
{
    state_machine_.REG_STATE(State::Initialization, handle_initialization, timeout_initialization_.get(), State::ListenToFirstPing);
    state_machine_.REG_STATE(State::ListenToFirstPing, handle_listen_first_ping, timeout_listen_firt_ping_.get(), State::BumpFlare);
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
    return State::ListenToFirstPing;
}

State FlareTask::handle_listen_first_ping()
{
    if (ping_found_) {
        ROS_INFO_STREAM("First ping was found");
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

    motion_.fix_heading(pinger_state_.heading, WaitMode::DONT_WAIT);

    switch (cur_zone_) {
    case Zone::Far: {
        motion_.thrust_forward(thrust_far_.get(), timeout_regul_.get(), WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Working in far zone");
        ROS_INFO_STREAM("\tCurrent pinger heading: " << pinger_state_.heading);
        ROS_INFO_STREAM("\tCurrent thrust: " << thrust_far_.get());
        break;
    }
    case Zone::Middle: {
        motion_.thrust_forward(thrust_middle_.get(), timeout_regul_.get(), WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Working in middle zone");
        ROS_INFO_STREAM("\tCurrent pinger heading: " << pinger_state_.heading);
        ROS_INFO_STREAM("\tCurrent thrust: " << thrust_middle_.get());
        break;
    }
    case Zone::Near: {
        libauv::Point2d thrusts = calc_thrust(navig_.last_head(), pinger_state_.heading);
        motion_.thrust_forward(thrusts.y, timeout_regul_.get(), WaitMode::DONT_WAIT);
        motion_.thrust_right(thrusts.x, timeout_regul_.get(), WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Working in near zone");
        ROS_INFO_STREAM("\tCurrent pinger heading: " << pinger_state_.heading);
        ROS_INFO_STREAM("\tCurrent thrust forward: " << thrusts.y);
        ROS_INFO_STREAM("\tCurrent thrust rightward: " << thrusts.x);
        break;
    }
    case Zone::Bump: {
        ROS_INFO_STREAM("Working in bump zone");
        return State::BumpFlare;
    }
    }

    ping_found_ = false;
    return State::GoToFlare;
}

State FlareTask::handle_bump_flare()
{
    // Я не уверен, но скорее всего, если написать turn_right(360),
    // то скорее всего просто ничего не призоизойдет
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

    ROS_INFO_STREAM("Current distance to pinger: " << msg.distance);
    pinger_state_ = msg;
    cur_zone_ = update_zone(msg);
    ping_found_ = true;
}

void FlareTask::init_ipc(ipc::Communicator& com)
{
    sub_ping_ = com.subscribe("dsp", &FlareTask::handle_pinger_found, this);
}

void FlareTask::init_zones()
{
    zones_.emplace_back(Zone::Far, far_border_.get(), infinity_.get());
    zones_.emplace_back(Zone::Middle, middle_border_.get(), far_border_.get());
    zones_.emplace_back(Zone::Near, close_border_.get(), middle_border_.get());
    zones_.emplace_back(Zone::Bump, 0, close_border_.get());
}

Zone FlareTask::update_zone(const dsp::MsgBeacon& msg)
{
    Zone zone;
    for (auto& z : zones_) {
        if (msg.distance < z.max && msg.distance >= z.min) {
            ++z.pings_in_row;
            if (z.pings_in_row >= pings_needed_.get() &&
                zone != z.zone) {
                zone = z.zone;
            }
        } else {
            z.pings_in_row = 0;
        }
        ROS_INFO_STREAM("Pings in a row: " << z.pings_in_row);
    }
    return zone;
}

libauv::Point2d FlareTask::calc_thrust(double cur_heading, double pinger_heading)
{
    libauv::Point2d h = MakePoint2(sin(cur_heading), cos(cur_heading));
    libauv::Point2d r = MakePoint2(h.y, -h.x);
    libauv::Point2d v = MakePoint2(sin(pinger_heading), cos(pinger_heading));
    v *= pinger_state_.distance * close_koef_.get();

    libauv::Point2d t = MakePoint2(v.x * r.x + v.y * r.y,
        v.x * h.x + v.y * h.y);

    double max_thrust = max_close_thrust_.get();

    limit_max_thrust(t.x, max_thrust);
    limit_max_thrust(t.y, max_thrust);
    ROS_INFO_STREAM("h: " << h);
    ROS_INFO_STREAM("r: " << r);
    ROS_INFO_STREAM("v: " << v);
    ROS_INFO_STREAM("t: " << t);

    return t;
}

void FlareTask::limit_max_thrust(double& thrust, double max_value)
{
    thrust = min(thrust, max_value);
    thrust = max(thrust, -max_value);
}

REGISTER_TASK(FlareTask, flare_task);