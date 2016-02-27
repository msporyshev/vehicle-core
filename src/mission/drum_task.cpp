#include "drum_task.h"

#include <algorithm>

#include <libauv/utils/math_u.h>

#include <ostream>
#include <algorithm>

DrumTask::DrumTask(const YamlReader& cfg, ipc::Communicator& com): Task<State>(cfg, com, State::Initialization)
{
    state_machine_.REG_STATE(State::Initialization, handle_initialization,
        timeout_initialization_.get(), State::ListenToFirstPing);

    state_machine_.REG_STATE(State::ListenToFirstPing, handle_listen_first_ping,
        timeout_listen_first_ping_.get(), State::BucketFindingInit);

    state_machine_.REG_STATE(State::GoToPinger, handle_go_pinger,
        timeout_go_pinger_.get(), State::BucketFindingInit);

    state_machine_.REG_STATE(State::BucketFindingInit, handle_bucket_finding_init,
        timeout_bucket_finding_init_.get(), State::FindBucket);

    state_machine_.REG_STATE(State::FindBucket, handle_find_bucket,
        timeout_find_bucket_.get(), State::ActiveSearching);

    state_machine_.REG_STATE(State::ActiveSearching, handle_active_searching,
        timeout_active_searching_.get(), State::DropBall);

    state_machine_.REG_STATE(State::StabilizeBucket, handle_stabilize_bucket,
        timeout_stabilize_bucket_.get(), State::DropBall);

    state_machine_.REG_STATE(State::DropBall, handle_drop_ball,
        timeout_drop_ball_.get(), State::Finalize);

    state_machine_.REG_STATE(State::Finalize, handle_finalize,
        timeout_finalize_.get(), State::Terminal);

    init_ipc(com);
    init_zones();
}

void DrumTask::init_ipc(ipc::Communicator& com)
{
    subscribe_ping_   = com.subscribe("dsp", &DrumTask::handle_ping, this);
    subscribe_circle_ = com.subscribe("video", &DrumTask::handle_circle_found, this);
}

State DrumTask::handle_initialization()
{
    ROS_DEBUG_STREAM("Dive into deep water. New depth: "
        << start_depth_.get() << ", previous depth: " << navig_.last_depth());

    motion_.fix_pitch();
    motion_.fix_depth(start_depth_.get());
    motion_.fix_heading(navig_.last_head());

    cmd_.set_dsp_mode(dsp::CommandType::Freq37500);

    ROS_INFO_STREAM("Initialization has been completed");

    return State::ListenToFirstPing;
}

State DrumTask::handle_listen_first_ping()
{
    if (ping_found_) {
        ROS_INFO_STREAM("First ping was found, bearing: " << pinger_state_.bearing);
        return State::GoToPinger;
    }

    double last_head = navig_.last_head();
    motion_.fix_heading(normalize_degree_angle(navig_.last_head() + heading_delta_.get()));

    ROS_INFO_STREAM("Pinger hasn't ever been heard. Heading was changed from " << last_head <<
        " to " << navig_.last_head());

    return State::ListenToFirstPing;
}

State DrumTask::handle_go_pinger()
{
    if(!ping_found_) {
        return State::GoToPinger;
    }

    ping_found_ = false;

    double heading = filter_pinger_heading(pinger_state_.heading);
    motion_.fix_heading(heading, WaitMode::DONT_WAIT);
    ROS_DEBUG_STREAM("Fixed heading after filtering: " << heading);

    if(cur_zone_ == Zone::Close) {
        return State::BucketFindingInit;
    }

    double thrust = get_zone_thrust(cur_zone_);
    motion_.thrust_forward(thrust, timeout_regul_.get(), WaitMode::DONT_WAIT);
    ROS_DEBUG_STREAM("Forward thrust: " << thrust);

    return State::GoToPinger;
}

State DrumTask::handle_bucket_finding_init()
{
    searching_timer_start_ = ros::Time(0.001);

    motion_.thrust_forward(0, timeout_regul_.get(), WaitMode::DONT_WAIT);

    cmd_.set_recognizers(Camera::Bottom, {"drum"});

    return State::FindBucket;
}

State DrumTask::handle_find_bucket()
{
    if(!drum_found_) {
        return State::FindBucket;
    }

    finding_count_++;
    ROS_DEBUG_STREAM("Finding bucket count: " << finding_count_);
    drum_found_ = false;

    if(finding_count_ >= finding_count_needed_.get()) {
        ROS_INFO_STREAM("Bucket finding success!");
        return State::StabilizeBucket;
    }

    return State::FindBucket;
}

State DrumTask::handle_active_searching()
{
    if(drum_found_) {
        ROS_INFO_STREAM("Bucket was found in active searching");
        drum_found_ = false;
        motion_.thrust_forward(0, timeout_regul_.get(), WaitMode::DONT_WAIT);
        motion_.thrust_right(0, timeout_regul_.get(), WaitMode::DONT_WAIT);
        return State::FindBucket;
    }

    if(ros::Time::now() > searching_timer_start_ + ros::Duration(active_searching_step_timeout_.get())) {
        searching_timer_start_ = ros::Time::now();
        searching_sub_state_++;
        ROS_INFO_STREAM("New searching substate: " << searching_sub_state_);
    } else {
        return State::ActiveSearching;
    }

    //Возможно более эффективно вставить сюда движение по спирали, но надо думать как это запрограммировать
    switch (searching_sub_state_) {
    case 1: {
        motion_.thrust_forward(active_searching_thrust_.get(),
        active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    case 2: {
        motion_.thrust_right(active_searching_thrust_.get(),
        active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    case 3: {
        motion_.thrust_backward(active_searching_thrust_.get(),
        2 * active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    case 4: {
        motion_.thrust_left(active_searching_thrust_.get(),
        2 * active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    case 5: {
        motion_.thrust_forward(active_searching_thrust_.get(),
        2 * active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    case 6: {
        motion_.thrust_right(active_searching_thrust_.get(),
        active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
        break;
    }
    default: {
        return State::DropBall;
    }
    }

    return State::ActiveSearching;
}

State DrumTask::handle_stabilize_bucket()
{
    if(!drum_found_) {
        return State::StabilizeBucket;
    }
    drum_found_ = false;

    if(stabilize()) {
        stabilize_count_++;
        ROS_DEBUG_STREAM("Stabilize bucket count: " << stabilize_count_);
    } else {
        stabilize_count_ = 0;
        ROS_DEBUG_STREAM("Stabilize bucket count was reseted");
    }

    if(stabilize_count_ >= stabilize_count_needed_.get()) {
        ROS_INFO_STREAM("Drum stabilization success!");
        return State::DropBall;
    }

    return State::StabilizeBucket;
}

State DrumTask::handle_drop_ball()
{
    ROS_INFO_STREAM("Dive for cargo drop " << drop_depth_.get() << " meters");

    motion_.fix_depth(drop_depth_.get());
    cmd_.drop_cargo();
    ros::Duration(timeout_sleep_.get()).sleep();

    return State::Finalize;
}

State DrumTask::handle_finalize()
{
    motion_.fix_heading(navig_.last_head());
    motion_.fix_depth(end_depth_.get());
    ROS_INFO_STREAM("Finalize stage has been completed with heading: " << navig_.last_head());
    return State::Terminal;
}

double DrumTask::get_zone_thrust(Zone zone)
{
    ROS_INFO_STREAM("Zone: "<< zone_typename.at(zone)
        <<", bearing: " << pinger_state_.bearing
        << ", dist: " << pinger_state_.distance);

    switch (zone) {
    case Zone::Far: {
        return thrust_far_.get();
    }
    case Zone::Middle: {
        return thrust_middle_.get();
    }
    case Zone::Near: {
        return thrust_near_.get();
    }
    }
}

bool DrumTask::stabilize()
{
    auto drum_center = bottom_camera_.frame_coord(drum_state_.center);

    ROS_DEBUG_STREAM("Drum undistorted center: " << drum_center.x << ", " << drum_center.y <<
        ", eps: " << stabilization_eps_.get());

    libauv::Point2d thrust_vector = get_new_thrust(drum_center);

    double thrust_forward = thrust_vector.y;
    double thrust_right = thrust_vector.x;

    motion_.thrust_forward(thrust_forward, timeout_regul_.get(), WaitMode::DONT_WAIT);
    motion_.thrust_right(thrust_right, timeout_regul_.get(), WaitMode::DONT_WAIT);

    ROS_DEBUG_STREAM("Stabilization thrust forward: " << thrust_forward);
    ROS_DEBUG_STREAM("Stabilization thrust right: " << thrust_right);

    return (norm(drum_center) <= stabilization_eps_.get());
}

libauv::Point2d DrumTask::get_new_thrust(libauv::Point2d drum_center)
{
    libauv::Point2d stab_thrust_p = drum_center * stab_coef_p_.get();
    return stab_thrust_p;
}

void DrumTask::init_zones()
{
    zones_.emplace_back(Zone::Far, far_border_.get(), infinity_.get());
    zones_.emplace_back(Zone::Middle, middle_border_.get(), far_border_.get());
    zones_.emplace_back(Zone::Near, close_border_.get(), middle_border_.get());
    zones_.emplace_back(Zone::Close, 0, close_border_.get());
}

double DrumTask::median_filter(std::vector<double> data)
{
    nth_element(data.begin(), data.begin() + data.size() / 2, data.end());
    return data[data.size() / 2];
}

double DrumTask::filter_pinger_heading(double heading)
{
    filtered_heading_.push_back(heading);

    if(filtered_heading_.size() <= filtered_heading_size_.get()) {
        return navig_.last_head();
    } else {
        filtered_heading_.erase(filtered_heading_.begin());
        return median_filter(filtered_heading_);
    }
}

void DrumTask::handle_ping(const dsp::MsgBeacon& msg)
{
    if (msg.beacon_type != 1) {
        ROS_WARN_STREAM("Signal from incorrect pinger. Ignore.");
        return;
    }

    pinger_state_ = msg;
    cur_zone_ = update_zone(msg);
    ping_found_ = true;

    ROS_DEBUG_STREAM("Ping was found! bearing: " << msg.bearing << ", heading: " << msg.heading
        << ", distance: " <<  msg.distance << ", type: " << msg.beacon_type);
}

void DrumTask::handle_circle_found(const video::MsgFoundCircle& msg)
{
    ROS_DEBUG_STREAM("Was found " << msg.circles.size() << "circles");

    double max_radius = 0;
    for(auto &circle: msg.circles) {
        if(circle.radius > max_radius) {
            drum_state_ = circle;
            max_radius = circle.radius;
        }
    }
    drum_found_ = true;

    ROS_DEBUG_STREAM("Drum was found! Center: " << drum_state_.center.x << ", " << drum_state_.center.y
        << ", R: " << drum_state_.radius << "frame #" << msg.frame_number);
}

Zone DrumTask::update_zone(const dsp::MsgBeacon& msg)
{
    Zone zone;
    for (auto& z : zones_) {
        if (msg.distance < z.max && msg.distance >= z.min) {
            ++z.pings_in_row;
            if (z.pings_in_row >= pings_needed_.get() &&
                zone != z.zone) {
                zone = z.zone;
                ROS_INFO_STREAM("Set new zone state: " << zone_typename.at(zone));
            }
        } else {
            z.pings_in_row = 0;
        }
        ROS_DEBUG_STREAM("Pings in a row: " << z.pings_in_row);
    }
    return zone;
}

REGISTER_TASK(DrumTask, drum_task);