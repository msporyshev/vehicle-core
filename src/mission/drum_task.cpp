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
        timeout_listen_firt_ping_.get(), State::FindBucket);
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
    subscribe_ping_ = com.subscribe("dsp", &DrumTask::handle_ping, this);
    subscribe_drum_ = com.subscribe("video", &DrumTask::handle_drum_found, this);

    key_send_pub_ = com.advertise_cmd<supervisor::CmdDeviceKey>("supervisor"); 
}

State DrumTask::handle_initialization()
{
    float dest_depth = start_depth_.get();
    
    motion_.fix_pitch();
    motion_.fix_depth(dest_depth);
    motion_.fix_heading(navig_.last_head());

    ROS_INFO_STREAM("Initialization has been completed. New depth: " << dest_depth << 
        ", previous depth: " << navig_.last_depth());
    return State::ListenToFirstPing;
}

State DrumTask::handle_listen_first_ping()
{
    if (ping_found_) {
        ROS_INFO_STREAM("First ping was found, bearing: " << pinger_state_.bearing);
        return State::GoToPinger;
    } 

    double last_head = navig_.last_head();
    motion_.fix_heading(normalize_degree_angle(last_head + heading_delta_.get()), heading_delta_timeout_.get());
    ROS_INFO_STREAM("Pinger hasn't ever been heard. Heading was changed from " << last_head << " to " << navig_.last_head());
    return State::ListenToFirstPing;
}

void DrumTask::config_vehicle_thrust(Zone zone)
{
    float thrust;
    std::string zone_name;

    switch (zone) {
        case Zone::Far:     { thrust = thrust_far_.get();    break; }
        case Zone::Middle:  { thrust = thrust_middle_.get(); break; }
        case Zone::Near:    { thrust = thrust_near_.get();   break; }
    }

    ROS_INFO_STREAM("Zone: "<< zone_typename.at(zone) <<", bearing: " << pinger_state_.bearing << ", dist: " \
        << pinger_state_.distance);

    motion_.thrust_forward(thrust, timeout_regul_.get(), WaitMode::DONT_WAIT);
}

State DrumTask::handle_go_pinger()
{
    if(!ping_found_) {
        return State::GoToPinger;
    } else {
        ping_found_ = false;
    }

    double heading = filter_pinger_heading(pinger_state_.heading);
    motion_.fix_heading(heading, WaitMode::DONT_WAIT);
    
    if(cur_zone_ == Zone::Close) {
        return State::FindBucket;
    } else {
        config_vehicle_thrust(cur_zone_);
    }

    return State::GoToPinger;
}

State DrumTask::handle_bucket_finding_init()
{
    motion_.thrust_forward(0, timeout_regul_.get(), WaitMode::DONT_WAIT);
    motion_.fix_heading(navig_.last_head());

    cmd_.set_recognizers(Camera::Bottom, {"drum"});

    return State::FindBucket;
}

State DrumTask::handle_find_bucket()
{
    static int finding_count = 0;

    if(!drum_found_) {
        return State::FindBucket;
    } else {
        finding_count++;
        drum_found_ = false;
    }

    if(finding_count >= finding_count_needed_.get()) {
        ROS_INFO_STREAM("Bucket finding success!");
        return State::StabilizeBucket;
    }

    return State::FindBucket;
}

State DrumTask::handle_active_searching()
{
    static int sub_state = 0;
    static ros::Time timer_start(0.001);

    if(ros::Time::now() > timer_start + ros::Duration(active_searching_step_timeout_.get())) {
        timer_start = ros::Time::now();
        sub_state++;
    } else {
        return State::ActiveSearching;
    }

    switch (sub_state) {
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
            active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
            break;
        }
        case 4: {
            motion_.thrust_left(active_searching_thrust_.get(), 
            active_searching_step_timeout_.get(), WaitMode::DONT_WAIT);
            break;
        }
        case 5: {
            return State::DropBall;
        }
    }

    return State::ActiveSearching;
}

State DrumTask::handle_stabilize_bucket()
{
    static int stabilize_count = 0;

    if(!drum_found_) {
        return State::StabilizeBucket;
    } else {
        drum_found_ = false;
    }

    if(stabilize()) {
        stabilize_count++;
    } else {
        stabilize_count = 0;
    }
    
    if(stabilize_count >= stabilize_count_needed_.get()) {
        ROS_INFO_STREAM("Drum stabilization success!");
        return State::DropBall;
    }

    return State::StabilizeBucket;
}

State DrumTask::handle_drop_ball()
{
    supervisor::CmdDeviceKey msg;

    motion_.move_down(drop_ball_deep_.get());

    msg.id = static_cast<int>(SupervisorDevices::Cargo_1);
    msg.state = true;
    key_send_pub_.publish(msg);
    
    ros::Duration(1).sleep();

    msg.state = false;
    key_send_pub_.publish(msg);

    return State::Finalize;
}

State DrumTask::handle_finalize()
{
    motion_.fix_heading(navig_.last_head());
    motion_.thrust_backward(thrust_finalize_.get(), timeout_finalize_.get());
    ROS_INFO_STREAM("Finalize stage has been completed with heading: " << navig_.last_head()
        << " and thrust " << thrust_finalize_.get());
    return State::Terminal;
}

bool DrumTask::stabilize()
{
    // //Жёстко прописаны размеры кадра, по идее надо их как то динамически получать
    int offset_forward = drum_state_.center.x - 400 / 2;
    int offset_right = drum_state_.center.y - 300 / 2;

    int offset_vector = sqrt(offset_forward * offset_forward + offset_right * offset_right);

    double thrust_forward = get_new_thrust(offset_forward, 0, coef_p_.get());
    double thrust_right   = get_new_thrust(offset_right, 0, coef_p_.get());

    motion_.thrust_forward(thrust_forward, timeout_regul_.get(), WaitMode::DONT_WAIT);
    motion_.thrust_forward(thrust_right, timeout_regul_.get(), WaitMode::DONT_WAIT);

    ROS_DEBUG_STREAM("Stabilization thrust forward: " << thrust_forward);
    ROS_DEBUG_STREAM("Stabilization thrust right: " << thrust_right);
    ROS_DEBUG_STREAM("Offset vector: " << thrust_right << ", eps: " << stabilization_vector_eps_.get());

    return (offset_vector <= stabilization_vector_eps_.get());
}

double DrumTask::get_new_thrust(double current_dist, double desired_dist, double coef_p)
{
    double err = current_dist - desired_dist;
    double stab_thrust_p = err * coef_p;

    if (stab_thrust_p > stab_thrust_p_limit_.get()) {
        stab_thrust_p = stab_thrust_p_limit_.get();
    }
    return stab_thrust_p;
}

void DrumTask::init_zones()
{
    zones_.emplace_back(Zone::Far, far_border_.get(), infinity_.get());
    zones_.emplace_back(Zone::Middle, middle_border_.get(), far_border_.get());
    zones_.emplace_back(Zone::Near, close_border_.get(), middle_border_.get());
    zones_.emplace_back(Zone::Close, 0, close_border_.get());
}

double median_filter(std::vector<double> data)
{
    sort(data.begin(), data.end());
    return data[static_cast<size_t>(data.size() / 2)];
}

double DrumTask::filter_pinger_heading(double heading)
{
    filtered_heading.push_back(heading);

    if(filtered_heading.size() <= filtered_heading_size_.get()) {
        return navig_.last_head();
    } else {
        filtered_heading.erase(filtered_heading.begin());
        return median_filter(filtered_heading);
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
}

void DrumTask::handle_drum_found(const video::MsgFoundDrum& msg)
{
    drum_state_ = msg;
    drum_found_ = true;

    ROS_INFO_STREAM("Drum was found! Center: " << msg.center.x << ", " << msg.center.y << ", R: " << msg.radius);
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