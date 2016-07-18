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
        BearingTargeting,
        CoordinatesTargeting,
        Finalize,
        Terminal
    };
}

double get_median(std::vector<double> values)
{
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
}

class PingerTask: public Task<State>
{
public:
    PingerTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::BearingTargeting);
        state_machine_.REG_STATE(State::BearingTargeting, handle_bearing_targeting,
            timeout_bearing_targeting_.get(), State::Finalize);
        state_machine_.REG_STATE(State::CoordinatesTargeting, handle_coordinates_targeting,
            timeout_coordinates_targeting_.get(), State::Finalize);
        state_machine_.REG_STATE(State::Finalize, handle_finalize,
            timeout_finalize_.get(), State::Terminal);
        
        pinger_headings_.resize(filter_size_.get());

        dsp_sub_ = comm.subscribe("dsp", &PingerTask::handle_pinger_found, this);
        pinger_pos_sub_ = comm.subscribe("mission", &PingerTask::handle_pinger_pos, this);
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("fix heading: " << odometry_.head());
        motion_.fix_pitch();
        motion_.fix_heading(odometry_.head());
        motion_.fix_depth(start_depth_.get());
        
        if(targeting_type_.get() == "coordinate") {
            return State::CoordinatesTargeting;    
        } else {
            return State::BearingTargeting;
        }
    }

    State handle_bearing_targeting() {

        if(!ping_found_) {
            return State::BearingTargeting;
        }

        ping_found_ = false;

        motion_.fix_heading(cur_heading_, WaitMode::DONT_WAIT);
        ROS_DEBUG_STREAM("Fixed heading after filtering: " << cur_heading_);

        double thrust = cur_dist_ > distance_threshold_.get() ? far_forward_thrust_.get():
                                                                near_forward_thrust_.get();
        motion_.thrust_forward(thrust, forward_thrust_timeout_.get(), WaitMode::DONT_WAIT);
        
        return State::BearingTargeting;
    }

    State handle_coordinates_targeting() {

        if(!coordinate_received_) {
            return State::BearingTargeting;
        }

        Point2d position = Point2d(current_pinger_position_.position.north, current_pinger_position_.position.east);
        motion_.fix_position(position, MoveMode::CRUISE, forward_thrust_timeout_.get(), WaitMode::DONT_WAIT);

        return State::BearingTargeting;
    }

    State handle_finalize() {

        motion_.fix_depth(0.5);
        return State::Terminal;
    }
    
    void handle_pinger_found(const dsp::MsgBeacon& msg)
    {
        ROS_INFO_STREAM("Ping received.");
        
        pinger_headings_[count_++ % filter_size_.get()] = msg.heading;

        cur_heading_ = use_median_.get() ? get_median(pinger_headings_) : msg.heading;
        cur_dist_ = msg.distance;
        ping_found_ = true;
    }

    void handle_pinger_pos(const mission::MsgPingerPosition& msg)
    {
        ROS_INFO_STREAM("Ping position received.");
        
        current_pinger_position_ = msg;
        coordinate_received_ = true;
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_bearing_targeting_);
    AUTOPARAM(double, timeout_coordinates_targeting_);
    AUTOPARAM(double, timeout_finalize_);

    AUTOPARAM(std::string, targeting_type_);

    AUTOPARAM(double, start_depth_);

    AUTOPARAM(double, distance_threshold_);
    AUTOPARAM(double, forward_thrust_timeout_);
    AUTOPARAM(double, far_forward_thrust_);
    AUTOPARAM(double, near_forward_thrust_);
    
    AUTOPARAM(int, filter_size_);
    AUTOPARAM(bool, use_median_);

    std::vector<double> pinger_headings_;

    ipc::Subscriber<dsp::MsgBeacon> dsp_sub_;
    ipc::Subscriber<mission::MsgPingerPosition> pinger_pos_sub_;

    bool ping_found_ = false;
    bool coordinate_received_ = false;

    mission::MsgPingerPosition current_pinger_position_;
    
    int count_ = 0;
    
    double cur_heading_ = 0;
    double cur_dist_ = 0;
};

REGISTER_TASK(PingerTask, pinger_task);