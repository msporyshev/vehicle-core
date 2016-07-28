#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>
#include <mission/MsgPingerPosition.h>

#include <point/point.h>
#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
    enum class State
    {
        Initialization,
        ListenFirstPing,
        BearingTargeting,
        CoordinatesTargeting,
        Finalize,
        Terminal
    };
}

class PingerTask: public Task<State>
{
public:
    PingerTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::ListenFirstPing);
        state_machine_.REG_STATE(State::ListenFirstPing, handle_listen_first_ping,
            timeout_listen_pings_.get(), State::BearingTargeting);
        state_machine_.REG_STATE(State::BearingTargeting, handle_bearing_targeting,
            timeout_bearing_targeting_.get(), State::CoordinatesTargeting);
        state_machine_.REG_STATE(State::CoordinatesTargeting, handle_coordinates_targeting,
            timeout_coordinates_targeting_.get(), State::Finalize);
        state_machine_.REG_STATE(State::Finalize, handle_finalize,
            timeout_finalize_.get(), State::Terminal);

        vehicle_depth_ = fabs(pinger_depth_.get() - near_zone_distance_.get());
        if(vehicle_depth_ >= minimum_depth_.get()) {
            ROS_INFO_STREAM("Calculated vehicle depth: " << vehicle_depth_);
        } else {
            vehicle_depth_ = minimum_depth_.get();
            ROS_WARN_STREAM("Calculated vehicle depth < minimum!. Set default: " << vehicle_depth_);
        }

        timer_update_ = comm.create_timer(period_pinger_emit_.get(), &PingerTask::weight_update, this);
        timer_rotate_ = comm.create_timer(period_vehicle_rotating_.get(), &PingerTask::rotation_update, this);

        dsp_sub_ = comm.subscribe("dsp", &PingerTask::handle_pinger_found, this);
        pinger_pos_sub_ = comm.subscribe("mission", &PingerTask::handle_pinger_pos, this);
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("Start heading: " << odometry_.head());
        
        start_heading_ = odometry_.head();
        motion_.fix_pitch();
        motion_.fix_heading(start_heading_);
        motion_.fix_depth(vehicle_depth_);

        enable_rotation = true;
        return State::ListenFirstPing;
    }

    State handle_listen_first_ping()
    {
        if(ping_found_) {
            return State::BearingTargeting;
        }

        if(is_need_rotate) {
            double heading = normalize_degree_angle(start_heading_ + heading_increment_);
            ROS_INFO_STREAM("Current heading: " << cur_pinger_.heading);
            motion_.fix_heading(heading);
            is_need_rotate = false;
        }

        return State::ListenFirstPing;
    }

    State handle_bearing_targeting() {

        if(!weight_update_) {
            return State::BearingTargeting;
        }
        weight_update_ = false;


        motion_.fix_heading(cur_pinger_.heading, WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Fixed heading: " << cur_pinger_.heading);

        motion_.thrust_forward(bearing_thrust_.get(), period_vehicle_moving_.get(), WaitMode::DONT_WAIT);

        if(weighted_pinger_.distance < near_zone_distance_.get()) {
            ROS_INFO_STREAM("Near zone distance success: " << weighted_pinger_.distance);
            return State::CoordinatesTargeting;
        }

        return State::BearingTargeting;
    }

    State handle_coordinates_targeting() {
        motion_.fix_position(odometry_.pos(), MoveMode::HOVER, timeout_coordinate_.get(), WaitMode::WAIT);
        ros::Duration(2).sleep();
        return State::Finalize;
    }

    State handle_finalize() {
        double current_distance = pow(odometry_.pos().north, 2) + pow(odometry_.pos().east, 2);
        next_branch_ = current_distance > pinger_distance_threshold_.get() ? "octagon" : "bins";
        
        if(next_branch_ == "octagon") {
            motion_.fix_depth(0, timeout_lift_up_.get());
        }

        return State::Terminal;
    }

    void handle_pinger_found(const dsp::MsgBeacon& msg)
    {
        ROS_INFO_STREAM("Ping received.");

        cur_pinger_ = msg;
        ping_found_ = true;
    }

    void handle_pinger_pos(const mission::MsgPingerPosition& msg)
    {

    }

    void weight_update(const ros::TimerEvent& event) {

        double w_coef = data_weight_.get();

        double weight           = weight_old_ * (1 - w_coef) + (ping_found_ ? w_coef : 0);
        double weight_dist      = weight_dist_old_ * (1 - w_coef) + cur_pinger_.distance * (ping_found_ ? w_coef : 0);

        weight_old_         = weight;
        weight_dist_old_    = weight_dist;

        weighted_pinger_.distance = weight_dist / weight;

        if(ping_found_) {
            ping_found_ = false;
            weight_update_ = true;
            ROS_INFO_STREAM("Weighted distance: " << weighted_pinger_.distance);
        } else {
            ROS_WARN_STREAM("New ping was not receive");
        }
    }

    void rotation_update(const ros::TimerEvent& event) {
        if(enable_rotation) {
            heading_increment_ += delta_rotating_.get();
            is_need_rotate = true;
        }
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_listen_pings_);
    AUTOPARAM(double, timeout_bearing_targeting_);
    AUTOPARAM(double, timeout_coordinates_targeting_);
    AUTOPARAM(double, timeout_finalize_);

    AUTOPARAM(double, period_pinger_emit_);
    AUTOPARAM(double, period_vehicle_moving_);
    AUTOPARAM(double, period_vehicle_rotating_);

    AUTOPARAM(double, timeout_lift_up_);
    
    AUTOPARAM(double, delta_rotating_);

    AUTOPARAM(double, pinger_depth_);
    AUTOPARAM(double, minimum_depth_);
    AUTOPARAM(double, near_zone_distance_);

    AUTOPARAM(double, bearing_thrust_);
    AUTOPARAM(double, timeout_coordinate_);
    
    AUTOPARAM(double, data_weight_);

    AUTOPARAM(double, pinger_distance_threshold_);

    ipc::Subscriber<dsp::MsgBeacon> dsp_sub_;
    ipc::Subscriber<mission::MsgPingerPosition> pinger_pos_sub_;

    bool ping_found_ = false;
    bool coordinate_received_ = false;
    bool weight_update_ = false;

    bool is_need_rotate = false;
    bool enable_rotation = false;

    ros::Timer timer_update_;
    ros::Timer timer_rotate_;

    double heading_increment_ = 0;

    dsp::MsgBeacon cur_pinger_;
    dsp::MsgBeacon weighted_pinger_;

    double start_heading_ = 0;
    double vehicle_depth_ = 0;

    double weight_old_ = 0;
    double weight_dist_old_ = 0;
};

REGISTER_TASK(PingerTask, pinger_task);