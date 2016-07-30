#include "task.h"
#include "task_factory.h"

#include <vision/MsgFoundGate.h>
#include <mission/MsgValidationGate.h>

#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
enum class State
{
    Initialization,
    LookingForGate,
    StabilizeGate,
    ProceedGate,
    Terminal
};
}

class NavigateChannelTask: public Task<State>
{
public:
    NavigateChannelTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::LookingForGate);
        state_machine_.REG_STATE(State::LookingForGate, handle_looking_for_gate,
            timeout_looking_for_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::StabilizeGate, handle_stabilize_gate,
            timeout_stabilize_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::ProceedGate, handle_proceed_gate,
            timeout_proceed_gate_.get(), State::Terminal);

        gate_sub_ = comm.subscribe("mission", &NavigateChannelTask::receive_gate, this);
    }

    State handle_initialization()
    {
        motion_.thrust_forward(thrust_initial_search_.get(), timeout_looking_for_gate_.get());

        ROS_INFO_STREAM("Initialization has been completed. Working on heading: " << odometry_.head()
            << ", depth: " << odometry_.depth().distance << ", thrust: " << thrust_initial_search_.get());

        cmd_.set_recognizers(Camera::Front, {"fargate"});

        return State::LookingForGate;
    }

    State handle_looking_for_gate()
    {
        return gate_found_ ? State::StabilizeGate : State::LookingForGate;
    }

    State handle_stabilize_gate()
    {
        if (!gate_found_) {
            return State::StabilizeGate;
        }

        if (large_count_ >= large_count_needed_.get()) {
            return State::ProceedGate;
        }

        double gate_heading = current_gate_.pos.direction;

        ROS_INFO_STREAM("New heading: " << gate_heading);
        motion_.fix_heading(gate_heading);
        ROS_INFO_STREAM("fix_heading completed");
        motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_gate_.get());
        gate_found_ = false;

        return State::StabilizeGate;
    }

    State handle_proceed_gate()
    {
        double head = odometry_.head();

        ROS_INFO("Fix current position");
        motion_.fix_position(odometry_.frame_pos(), MoveMode::HEADING_FREE, timeout_fix_position_.get());

        ROS_INFO("Move forward");
        motion_.move_forward(proceed_distance_forward_.get(), timeout_fix_position_.get());

        ROS_INFO("Fix current position");
        motion_.fix_position(odometry_.pos(), MoveMode::HEADING_FREE, timeout_fix_position_.get());

        // ROS_INFO("Turn left 90");
        // motion_.turn_left(90);

        double heading = odometry_.head();
        ROS_INFO("Style rotation");
        // motion_.turn_left(180);
        auto pos = odometry_.pos();
        double distance = proceed_distance_style_.get();
        pos.north += distance * cos(to_rad(heading));
        pos.east += distance * sin(to_rad(heading));

        ROS_INFO("Rotate style");
        motion_.fix_heading(heading + style_heading_delta_.get());


        // ROS_INFO("Move right");
        // motion_.move_right(proceed_distance_style_.get(), timeout_proceed_gate_.get());

        ROS_INFO("Move style");
        motion_.fix_position(pos, MoveMode::HOVER, timeout_proceed_gate_.get());

        motion_.fix_heading(heading + heading_delta_.get());
        // motion_.move_forward(current_gate_.pos.distance + distance_after_gate_.get(), timeout_proceed_gate_.get());
        motion_.thrust_forward(proceed_thrust_.get(), timeout_forward_.get(), WaitMode::WAIT);

        return State::Terminal;
    }

    void receive_gate(const mission::MsgValidationGate& msg)
    {
        odometry_.add_frame_odometry(msg.odometry);
        current_gate_ = msg;

        if (is_gate_large()) {
            large_count_++;
        } else {
            large_count_ = 0;
        }

        gate_found_ = true;
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_looking_for_gate_);
    AUTOPARAM(double, timeout_stabilize_gate_);
    AUTOPARAM(double, timeout_proceed_gate_);
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, thrust_initial_search_);

    AUTOPARAM(double, style_heading_delta_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, thrust_stabilize_);
    AUTOPARAM(double, timeout_fix_position_);
    AUTOPARAM(double, proceed_thrust_);
    AUTOPARAM(double, proceed_distance_forward_);
    AUTOPARAM(double, proceed_distance_style_);
    AUTOPARAM(double, gate_ratio_);
    AUTOPARAM(int, large_count_needed_);
    AUTOPARAM(double, gate_real_size_);
    AUTOPARAM(double, distance_after_gate_);
    AUTOPARAM(double, timeout_forward_);

    bool gate_found_ = false;
    int large_count_ = 0;
    int stabilize_count_ = 0;

    mission::MsgValidationGate current_gate_;

    ipc::Subscriber<mission::MsgValidationGate> gate_sub_;

    bool is_gate_large()
    {
        return current_gate_.width_ratio >= gate_ratio_.get();
    }
};

REGISTER_TASK(NavigateChannelTask, navigate_channel_task);