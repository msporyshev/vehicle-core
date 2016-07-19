#include "task.h"
#include "task_factory.h"

#include <mission/MsgBuoy.h>

#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
enum class State
{
    Initialization,
    LookingForGate,
    FixBuoy,
    FixLeftBar,
    Spin,
    Terminal
};
}

class BuoyTask: public Task<State>
{
public:
    BuoyTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::LookingForGate);
        state_machine_.REG_STATE(State::LookingForGate, handle_looking_for_gate,
            timeout_looking_for_gate_.get(), State::FixLeftBar);
        state_machine_.REG_STATE(State::FixBuoy, handle_stabilize_gate,
            timeout_stabilize_gate_.get(), State::FixLeftBar);
        state_machine_.REG_STATE(State::FixLeftBar, handle_fix_left_bar,
            timeout_fix_left_bar_.get(), State::Spin);
        state_machine_.REG_STATE(State::Spin, handle_spin,
            timeout_spin_.get(), State::Terminal);

        // gate_sub_ = comm.subscribe("vision", &BuoyTask::handle_gate_found, this);

        channel_sub_ = comm.subscribe("mission", &BuoyTask::receive_buoy, this);
    }

    void receive_buoy(const mission::MsgBuoy& msg)
    {
        odometry_.add_frame_odometry(msg.odometry);
        current_buoy_ = msg;
        buoy_found_ = true;
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("fix heading: " << odometry_.head());
        motion_.fix_pitch();
        motion_.fix_heading(odometry_.head());
        motion_.fix_depth(start_depth_.get());
        motion_.thrust_forward(thrust_initial_search_.get(), timeout_looking_for_gate_.get());

        ROS_INFO_STREAM("Initialization has been completed. Working on heading: " << odometry_.head()
            << ", depth: " << odometry_.depth().distance << ", thrust: " << thrust_initial_search_.get());

        cmd_.set_recognizers(Camera::Front, {"circle"});

        start_heading_ = odometry_.head();

        return State::LookingForGate;
    }

    State handle_looking_for_buoy()
    {
        return buoy_found_ ? State::FixBuoy : State::LookingForGate;
    }

    State handle_fix_buoy()
    {
        if (!buoy_found_) {
            return State::FixBuoy;
        }

        motion_.fix_heading(current_buoy_.direction, WaitMode::DONT_WAIT);

        if (large_count_ >= large_count_needed_.get()) {
            motion_.fix_position(odometry_.frame_pos(), timeout_fix_position_.get());
            buoy_spot_ = odometry_.frame_pos();
            return State::FixBuoyDepth;
        } else {
            motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_gate_.get());
        }

        double gate_heading = lost_gate_ ? start_heading_ : current_buoy_.direction;
        if (std::abs(gate_heading - start_heading_) >= heading_delta_.get()) {
            gate_heading = start_heading_;
        }

        buoy_found_ = false;

        return State::FixBuoy;
    }

    State handle_fix_buoy_depth()
    {
        if (!buoy_found_) {
            return State::FixBuoyDepth;
        }
        buoy_found_ = false;

        motion_.fix_heading(current_buoy_.direction, WaitMode::DONT_WAIT);
        motion_.fix_depth(current_buoy_.depth, WaitMode::DONT_WAIT);

        return State::FixBuoyDepth;
    }

    State handle_kick_buoy()
    {
        motion_.fix_heading(current_buoy_.direction);
        motion_.move_forward(current_buoy_.distance, timeout_move_forward_.get());
        motion_.fix_position(buoy_spot_, timeout_move_forward_.get());
        return State::FinalMove;
    }

    State handle_final_move()
    {
        motion_.move_forward(move_forward_distance_.get(), timeout_move_forward_.get());
        return State::Terminal;
    }

private:
    navig::MsgLocalPosition buoy_spot_;
    mission::MsgBuoy current_buoy_;

    // ipc::Subscriber<vision::MsgFoundGate> gate_sub_;
    ipc::Subscriber<mission::MsgBuoy> channel_sub_;
    ros::Publisher gate_pub_;

    bool is_gate_large()
    {
        return current_buoy_.width_ratio >= gate_ratio_.get();
    }
};

REGISTER_TASK(BuoyTask, buoy_task);