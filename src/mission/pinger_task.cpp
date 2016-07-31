#include "task.h"
#include "task_factory.h"

#include <dsp/MsgBeacon.h>

#include <point/point.h>
#include <utils/math_u.h>

#include <algorithm>
#include <vector>

using namespace utils;

namespace {
    enum class State
    {
        Initialization,
        ListenFirstPing,
        PingerListening,
        PingerSelection,
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
            timeout_listen_pings_.get(), State::PingerListening);

        state_machine_.REG_STATE(State::PingerListening, handle_pinger_listening,
            timeout_pinger_listening_.get(), State::PingerSelection);

        state_machine_.REG_STATE(State::PingerSelection, handle_pinger_selection,
            timeout_pinger_selection_.get(), State::BearingTargeting);
        
        state_machine_.REG_STATE(State::BearingTargeting, handle_bearing_targeting,
            timeout_bearing_targeting_.get(), State::CoordinatesTargeting);

        state_machine_.REG_STATE(State::CoordinatesTargeting, handle_coordinates_targeting,
            timeout_coordinates_targeting_.get(), State::Finalize);

        state_machine_.REG_STATE(State::Finalize, handle_finalize,
            timeout_finalize_.get(), State::Terminal);

        timer_rotate_ = comm.create_timer(period_vehicle_rotating_.get(), &PingerTask::rotation_update, this);

        dsp_sub_ = comm.subscribe("dsp", &PingerTask::handle_pinger_found, this);
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("Start heading: " << odometry_.head());

        start_heading_ = odometry_.head();
        motion_.fix_pitch();
        motion_.fix_heading(start_heading_);

        enable_rotation = true;
        return State::ListenFirstPing;
    }

    State handle_listen_first_ping()
    {
        if(ping_found_) {
            return State::PingerListening;
        }

        if(is_need_rotate) {
            double heading = normalize_degree_angle(start_heading_ + heading_increment_);
            ROS_INFO_STREAM("Current heading: " << cur_pinger_.heading);
            motion_.fix_heading(heading);
            is_need_rotate = false;
        }

        return State::ListenFirstPing;
    }


    State handle_pinger_listening()
    {
        if(ping_found_) {
            motion_.fix_heading(cur_pinger_.heading, WaitMode::DONT_WAIT);
            ping_found_ = false;
        }
        return State::PingerListening;
    }

    State handle_pinger_selection()
    {
        int vec_size = headings_array_.size();
        int size = std::min(vec_size, selection_window_size_.get());
        ROS_INFO_STREAM("Last headings size: " << size);
        
        std::vector<double> last_headings(size);

        std::copy(headings_array_.end() - size, headings_array_.end(), last_headings.begin());

        ROS_INFO_STREAM("Last heading: ");
        for(auto &el: last_headings) {
            ROS_INFO_STREAM("\t -" << el);
        }

        if(last_headings.size() == 0) {
            next_branch_ = "octagon";
            return State::Finalize;
        }

        double sum_sin = 0, sum_cos = 0;
        for(auto &heading: last_headings) {
            sum_sin += sin(to_rad(heading));
            sum_cos += cos(to_rad(heading));
        }

        double averge_pinger_heading = to_deg(atan2(sum_sin, sum_cos));
        ROS_INFO_STREAM("Average heading: " << averge_pinger_heading);
        
        double relative_gate_heading = normalize_degree_angle(averge_pinger_heading - start_heading_);
        ROS_INFO_STREAM("Relative heading: " << relative_gate_heading);
        
        if(relative_gate_heading > angle_threshold_.get()) {
            next_branch_ = "bins";
        } else {
            next_branch_ = "octagon";
        }
        
        return State::BearingTargeting;
    }

    State handle_bearing_targeting() {

        if(!ping_found_) {
            return State::BearingTargeting;
        }
        ping_found_ = false;


        motion_.fix_heading(cur_pinger_.heading, WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Fixed heading: " << cur_pinger_.heading);
        ROS_INFO_STREAM("Current distance: " << cur_pinger_.distance);

        double vehicle_thrust = 0;
        double timeout_thrust = 0;
        
        if(cur_pinger_.distance >= zone_threshold_distance_.get()) {
            vehicle_thrust = far_thrust_.get();
            timeout_thrust = far_thrust_period_.get();
            ROS_INFO_STREAM("Near zone, thrust: " << vehicle_thrust);
        } else {
            vehicle_thrust = near_thrust_.get();
            timeout_thrust = near_thrust_period_.get();
            ROS_INFO_STREAM("Far zone, thrust: " << vehicle_thrust);
        }

        if(cur_pinger_.distance >= lift_up_distance_.get()) {
            near_zone_conter_ = 0;
        } else {
            near_zone_conter_++;
        }
        ROS_INFO_STREAM("Zone counter: " << near_zone_conter_);

        if(near_zone_conter_ >= lift_up_count_.get()) {
            return State::CoordinatesTargeting;
        }

        motion_.thrust_forward(vehicle_thrust, timeout_thrust, WaitMode::DONT_WAIT);
        return State::BearingTargeting;
    }

    State handle_coordinates_targeting() {
        motion_.fix_position(odometry_.pos(), MoveMode::HOVER, timeout_coordinate_.get(), WaitMode::WAIT);
        return State::Finalize;
    }

    State handle_finalize() {
        if(next_branch_ == "octagon") {
            motion_.fix_depth(0, timeout_lift_up_.get());
        }

        return State::Terminal;
    }

    void handle_pinger_found(const dsp::MsgBeacon& msg)
    {
        ROS_INFO_STREAM("Ping received.");
        headings_array_.push_back(cur_pinger_.heading);
        cur_pinger_ = msg;
        ping_found_ = true;
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
    AUTOPARAM(double, timeout_pinger_listening_);
    AUTOPARAM(double, timeout_pinger_selection_);
    AUTOPARAM(double, timeout_bearing_targeting_);
    AUTOPARAM(double, timeout_coordinates_targeting_);
    AUTOPARAM(double, timeout_finalize_);

    AUTOPARAM(double, period_pinger_emit_);
    AUTOPARAM(double, period_vehicle_rotating_);

    AUTOPARAM(double, timeout_lift_up_);

    AUTOPARAM(double, delta_rotating_);
 
    AUTOPARAM(double, lift_up_distance_);
    AUTOPARAM(double, zone_threshold_distance_);
    
    AUTOPARAM(double, far_thrust_);
    AUTOPARAM(double, near_thrust_);

    AUTOPARAM(double, far_thrust_period_);
    AUTOPARAM(double, near_thrust_period_);

    AUTOPARAM(double, lift_up_count_);

    AUTOPARAM(double, timeout_coordinate_);

    AUTOPARAM(double, angle_threshold_);
    AUTOPARAM(int, selection_window_size_);

    ipc::Subscriber<dsp::MsgBeacon> dsp_sub_;
    
    bool ping_found_ = false;

    bool is_need_rotate = false;
    bool enable_rotation = false;

    ros::Timer timer_rotate_;

    double heading_increment_ = 0;

    dsp::MsgBeacon cur_pinger_;

    double start_heading_ = 0;

    int near_zone_conter_ = 0;

    std::vector<double> headings_array_;
};

REGISTER_TASK(PingerTask, pinger_task);