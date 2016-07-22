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

class PingerTask: public Task<State>
{
public:
    PingerTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::BearingTargeting);
        state_machine_.REG_STATE(State::BearingTargeting, handle_bearing_targeting,
            timeout_bearing_targeting_.get(), State::CoordinatesTargeting);
        state_machine_.REG_STATE(State::CoordinatesTargeting, handle_coordinates_targeting,
            timeout_coordinates_targeting_.get(), State::Finalize);
        state_machine_.REG_STATE(State::Finalize, handle_finalize,
            timeout_finalize_.get(), State::Terminal);
        
        bearing_distance_ = pinger_depth_.get() - dive_depth_.get();
        if(bearing_distance_ < 0) {
            ROS_ASSERT_MSG(bearing_distance_ > 0, "bearing_distance_ < 0: %f", bearing_distance_);
        }

        timer_update_ = comm.create_timer(period_pinger_emit_.get(), &PingerTask::weight_update, this);

        dsp_sub_ = comm.subscribe("dsp", &PingerTask::handle_pinger_found, this);
        pinger_pos_sub_ = comm.subscribe("mission", &PingerTask::handle_pinger_pos, this);
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("Fix heading: " << odometry_.head());
        motion_.fix_pitch();
        motion_.fix_heading(odometry_.head());
        motion_.fix_depth(dive_depth_.get());
        
        if(targeting_type_.get() == "bearing") {
            ROS_INFO_STREAM("Targeting type: " << targeting_type_.get());
            return State::BearingTargeting;  
        } else {
            ROS_WARN_STREAM("Wrong targeting type!");
            return State::Finalize;
        }
    }

    State handle_bearing_targeting() {

        if(!weight_update_) {
            return State::BearingTargeting;
        }
        weight_update_ = false;


        motion_.fix_heading(weighted_pinger_.heading, WaitMode::DONT_WAIT);
        ROS_INFO_STREAM("Fixed heading: " << weighted_pinger_.heading);

        motion_.thrust_forward(bearing_thrust_.get(), period_pinger_emit_.get(), WaitMode::DONT_WAIT);
        
        if(weighted_pinger_.distance < bearing_distance_) {
            ROS_INFO_STREAM("Bearing distance success: " << weighted_pinger_.distance);
            return State::CoordinatesTargeting;
        }
        
        return State::BearingTargeting;
    }

    State handle_coordinates_targeting() {

        if(!weight_update_) {
            return State::BearingTargeting;
        }
        weight_update_ = false;

        double north = odometry_.pos().north;
        double east  = odometry_.pos().east;

        double pinger_north = north + weighted_pinger_.distance * cos(to_rad(weighted_pinger_.heading));
        double pinger_east  = east + weighted_pinger_.distance * sin(to_rad(weighted_pinger_.heading));

        Point2d position = Point2d(pinger_north, pinger_east);
        
        motion_.fix_position(position, MoveMode::HOVER, period_pinger_emit_.get(), WaitMode::DONT_WAIT);

        if(weighted_pinger_.distance < finalize_distance_.get()) {
            return State::Finalize;
        }

        return State::CoordinatesTargeting;
    }

    State handle_finalize() {

        motion_.fix_depth(0);
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
        double weight_heading   = weight_heading_old_ * (1 - w_coef) + cur_pinger_.heading * (ping_found_ ? w_coef : 0);
        double weight_dist      = weight_dist_old_ * (1 - w_coef) + cur_pinger_.distance * (ping_found_ ? w_coef : 0);

        weight_old_         = weight;
        weight_heading_old_ = weight_heading;
        weight_dist_old_    = weight_dist;

        weighted_pinger_.heading  = weight_heading / weight;
        weighted_pinger_.distance = weight_dist / weight;

        if(ping_found_) {
            ping_found_ = false;
            weight_update_ = true;

            ROS_INFO_STREAM("Weighted heading: " << weighted_pinger_.heading);
            ROS_INFO_STREAM("Weighted distance: " << weighted_pinger_.distance);
        } else {
            ROS_WARN_STREAM("New ping is not received");
        }
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_bearing_targeting_);
    AUTOPARAM(double, timeout_coordinates_targeting_);
    AUTOPARAM(double, timeout_finalize_);

    AUTOPARAM(std::string, targeting_type_);

    AUTOPARAM(double, period_pinger_emit_);

    AUTOPARAM(double, pinger_depth_);
    AUTOPARAM(double, dive_depth_);

    AUTOPARAM(double, finalize_distance_);
    AUTOPARAM(double, bearing_thrust_);
    
    AUTOPARAM(double, data_weight_);
    
    ipc::Subscriber<dsp::MsgBeacon> dsp_sub_;
    ipc::Subscriber<mission::MsgPingerPosition> pinger_pos_sub_;

    bool ping_found_ = false;
    bool coordinate_received_ = false;
    bool weight_update_ = false;

    mission::MsgPingerPosition current_pinger_position_;

    ros::Timer timer_update_;

    dsp::MsgBeacon cur_pinger_;
    dsp::MsgBeacon weighted_pinger_;
    
    double bearing_distance_ = 0;

    double weight_old_ = 0;
    double weight_heading_old_ = 0;
    double weight_dist_old_ = 0;
};

REGISTER_TASK(PingerTask, pinger_task);