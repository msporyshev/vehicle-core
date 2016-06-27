#include "flare_vision_task.h"

#include <algorithm>

#include <libauv/utils/math_u.h>

#include <ostream>
#include <algorithm>

using namespace utils;

namespace {

double get_median(std::vector<double> values)
{
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
}

}

FlareVisionTask::FlareVisionTask(const YamlReader& cfg, ipc::Communicator& com): Task<State>(cfg, com, State::Initialization)
    ,pinger_headings_(filter_size_.get())
{
    state_machine_.REG_STATE(State::Initialization, handle_initialization, timeout_initialization_.get(), State::ListenToFirstPing);
    state_machine_.REG_STATE(State::GoToFlare, handle_go_flare, timeout_go_flare_.get(), State::BumpFlare);
    state_machine_.REG_STATE(State::BumpFlare, handle_bump_flare, timeout_bump_flare_.get(), State::Finalize);
    state_machine_.REG_STATE(State::Finalize, handle_finalize, timeout_finalize_.get(), State::Terminal);

    init_ipc(com);
    init_zones();
}

State FlareVisionTask::handle_initialization()
{
    motion_.fix_pitch();
    motion_.fix_depth(start_depth_.get());

    if (pinger_id_.get() == 0) {
        cmd_.set_dsp_mode(dsp::CommandType::Freq37500);
    } else {
        cmd_.set_dsp_mode(dsp::CommandType::Freq20000);
    }

    cmd_.set_recognizers(Camera::Front, {"stripe"});

    motion_.fix_heading(normalize_degree_angle(navig_.last_head() + turn_angle_.get()));
    motion_.thrust_forward(thrust_forward_.get(), timeout_thrust_forward_.get());
    ros::Duration(timeout_thrust_forward_.get()).sleep();
    ROS_INFO_STREAM("Initialization has been completed. Working on depth: " << navig_.last_depth());


    ping_found_ = false;
    count_ = 0;

    return State::ListenToFirstPing;
}

State FlareVisionTask::handle_go_flare()
{
    if (!ping_found_) {
        if (timestamp() - last_time_ > 1.0 && stripe_found_) {
            double stripe_head = get_new_head(center_);
            motion_.fix_heading(stripe_head, WaitMode::DONT_WAIT);
            motion_.thrust_forward(thrust_close_in_.get(), timeout_regul_.get(), WaitMode::DONT_WAIT);
            ROS_INFO_STREAM("Going by vision, heading: " << stripe_head);
        }
        stripe_found_ = false;
        return State::GoToFlare;
    }
    stripe_found_ = false;
    ping_found_ = false;

    ROS_INFO_STREAM("Heading to pinger: " << cur_heading_);
    motion_.fix_heading(cur_heading_, WaitMode::DONT_WAIT);

    // if (cur_zone_ == Zone::Bump) {
    //     ROS_INFO_STREAM("Working in bump zone");
    //     return State::BumpFlare;
    // }

    motion_.thrust_forward(thrust_close_in_.get(), timeout_regul_.get(), WaitMode::DONT_WAIT);
    ROS_INFO_STREAM("Working in CloseIn zone");
    ROS_INFO_STREAM("\tCurrent pinger heading: " << cur_heading_);
    ROS_INFO_STREAM("\tCurrent thrust: " << thrust_close_in_.get());

    return State::GoToFlare;
}

State FlareVisionTask::handle_bump_flare()
{
    motion_.fix_heading(normalize_degree_angle(navig_.last_head() + 179));
    motion_.fix_heading(normalize_degree_angle(navig_.last_head() + 179));
    return State::Finalize;
}

State FlareVisionTask::handle_finalize()
{
    motion_.fix_heading(navig_.last_head());
    motion_.thrust_backward(thrust_finalize_.get(), timeout_finalize_.get());
    ROS_INFO_STREAM("Finalize stage has been completed with heading: " << navig_.last_head()
        << " and thrust " << thrust_finalize_.get());
    return State::Terminal;
}

void FlareVisionTask::handle_pinger_found(const dsp::MsgBeacon& msg)
{
    if (msg.beacon_type != pinger_id_.get()) {
        ROS_INFO_STREAM("Signal from incorrect pinger. Ignore.");
        return;
    }

    pinger_headings_[count_++ % filter_size_.get()] = msg.heading;

    cur_heading_ = use_median_.get() ? get_median(pinger_headings_) : msg.heading;
    cur_dist_ = msg.distance;

    last_time_ = timestamp();

    ROS_INFO_STREAM("Pinger found");
    ROS_INFO_STREAM("Current distance to pinger: " << msg.distance);
    cur_zone_ = update_zone(msg);
    ping_found_ = true;
}

void FlareVisionTask::init_ipc(ipc::Communicator& com)
{
    sub_ping_ = com.subscribe("dsp", &FlareVisionTask::handle_pinger_found, this);
    sub_stripe_ = com.subscribe("vision", &FlareVisionTask::handle_stripe_found, this);
}

double FlareVisionTask::get_new_head(double center)
{
    double last_head = navig_.last_head();
    double angle = front_camera_.heading_to_point(MakePoint2(center, 0.));
    double new_head = R_to_DEG_ * normalize_angle(DEG_to_R_ * last_head + angle);

    ROS_INFO_STREAM("Last head = " << last_head << "\n");
    ROS_INFO_STREAM("Angle to object center = " << angle << "\n");
    ROS_INFO_STREAM("New head = " << new_head << "\n");

    return new_head;
}

void FlareVisionTask::handle_stripe_found(const vision::MsgFoundStripe& msg)
{
    auto stripe = msg.stripes.front();
    int x = stripe.begin.x;

    center_ = front_camera_.frame_coord(MakePoint2(x, 0)).x;

    ROS_INFO_STREAM("Stripe was found!" << std::endl);
    ROS_INFO_STREAM("Center: " << center_ << std::endl);

    stripe_found_ = true;
}

void FlareVisionTask::init_zones()
{
    zones_.emplace_back(Zone::CloseIn, close_in_border_.get(), infinity_.get());
    zones_.emplace_back(Zone::Bump, 0, close_in_border_.get());
}

Zone FlareVisionTask::update_zone(const dsp::MsgBeacon& msg)
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

REGISTER_TASK(FlareVisionTask, flare_vision_task);
