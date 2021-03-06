#include "task.h"
#include "task_factory.h"

#include <mission/MsgNavigateChannel.h>

#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
enum class State
{
    Initialization,
    LookingForGate,
    StabilizeGate,
    FixLeftBar,
    Spin,
    Terminal
};
}

class ChannelSpinTask: public Task<State>
{
public:
    ChannelSpinTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization,
            timeout_initialization_.get(), State::LookingForGate);
        state_machine_.REG_STATE(State::LookingForGate, handle_looking_for_gate,
            timeout_looking_for_gate_.get(), State::FixLeftBar);
        state_machine_.REG_STATE(State::StabilizeGate, handle_stabilize_gate,
            timeout_stabilize_gate_.get(), State::FixLeftBar);
        state_machine_.REG_STATE(State::FixLeftBar, handle_fix_left_bar,
            timeout_fix_left_bar_.get(), State::Spin);
        state_machine_.REG_STATE(State::Spin, handle_spin,
            timeout_spin_.get(), State::Terminal);

        channel_sub_ = comm.subscribe("mission", &ChannelSpinTask::receive_channel, this);
    }

    void receive_channel(const mission::MsgNavigateChannel& msg)
    {
        odometry_.add_frame_odometry(msg.odometry);
        current_channel_ = msg;

        if (is_gate_large()) {
            large_count_++;
        }
        gate_found_ = true;
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
        gate_found_ = false;

        if (large_count_ >= large_count_needed_.get()) {
            ROS_INFO_STREAM("Stabilize left bar, heading: " << current_channel_.left_pos.direction);
            motion_.fix_heading(current_channel_.left_pos.direction);

            ROS_INFO("Set recognizer to channel");
            cmd_.set_recognizers(Camera::Front, {"channel"});
            ROS_INFO_STREAM("Waiting timeout: " << timeout_regul_.get());
            ros::Duration(timeout_regul_.get()).sleep();

            return State::FixLeftBar;
        }

        double gate_heading = current_channel_.pos.direction;

        ROS_INFO_STREAM("New heading: " << gate_heading);
        motion_.fix_heading(gate_heading);
        ROS_INFO_STREAM("fix_heading completed");
        motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_gate_.get());

        return State::StabilizeGate;
    }

    State handle_fix_left_bar()
    {
        if (!gate_found_) {
            return State::FixLeftBar;
        }
        gate_found_ = false;

        ROS_INFO("Fix target");
        motion_.fix_target(current_channel_.left_pos.position,
            fix_distance_.get(), timeout_spin_.get(), WaitMode::DONT_WAIT);

        if (abs(current_channel_.pos.distance  - fix_distance_.get()) < distance_err_.get()) {
            start_spin_heading_ = odometry_.head();
            finish_spin_heading_ = normalize_degree_angle(start_spin_heading_ - spin_angle_.get());
            return State::Spin;
        }

        return State::FixLeftBar;
    }

    State handle_spin()
    {
        motion_.thrust_right(thrust_spin_.get(), timeout_thrust_spin_.get(), WaitMode::WAIT);
        if (abs(degree_angle_diff(finish_spin_heading_,  odometry_.head())) < heading_delta_.get()) {
            motion_.fix_heading(start_spin_heading_);
            return State::Terminal;
        }

        return State::Spin;
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_looking_for_gate_);
    AUTOPARAM(double, timeout_stabilize_gate_);

    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, thrust_initial_search_);

    AUTOPARAM(double, thrust_stabilize_);
    AUTOPARAM(double, timeout_fix_position_);
    AUTOPARAM(double, proceed_thrust_);
    AUTOPARAM(double, proceed_distance_forward_);
    AUTOPARAM(double, proceed_distance_right_);
    AUTOPARAM(double, gate_ratio_);
    AUTOPARAM(int, large_count_needed_);
    AUTOPARAM(double, distance_after_gate_);

    AUTOPARAM(double, timeout_fix_left_bar_);
    AUTOPARAM(double, timeout_spin_);
    AUTOPARAM(double, fix_distance_);
    AUTOPARAM(double, distance_err_);
    AUTOPARAM(double, spin_angle_);
    AUTOPARAM(double, thrust_spin_);
    AUTOPARAM(double, timeout_thrust_spin_);

    double finish_spin_heading_ = 0;
    double start_spin_heading_ = 0;

    bool gate_found_ = false;
    int large_count_ = 0;
    int stabilize_count_ = 0;
    int x1_ = 0;
    int x2_ = 0;

    mission::MsgNavigateChannel current_channel_;

    // ipc::Subscriber<vision::MsgFoundGate> gate_sub_;
    ipc::Subscriber<mission::MsgNavigateChannel> channel_sub_;
    ros::Publisher gate_pub_;

    bool is_gate_large()
    {
        return current_channel_.width_ratio >= gate_ratio_.get();
    }
};

REGISTER_TASK(ChannelSpinTask, channel_spin_task);