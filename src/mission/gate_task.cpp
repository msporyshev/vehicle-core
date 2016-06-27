#include "task.h"
#include "task_factory.h"

#include <vision/MsgFoundGate.h>

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

class GateTask: public Task<State>
{
public:
    GateTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Initialization)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization, timeout_initialization_.get(), State::LookingForGate);
        state_machine_.REG_STATE(State::LookingForGate, handle_looking_for_gate, timeout_looking_for_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::StabilizeGate, handle_stabilize_gate, timeout_stabilize_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::ProceedGate, handle_proceed_gate, timeout_proceed_gate_.get(), State::Terminal);

        init_ipc(comm);
    }

    State handle_initialization()
    {
        ROS_INFO_STREAM("fix heading: " << navig_.last_head());
        motion_.fix_pitch();
        start_heading_ = navig_.last_head();
        motion_.fix_heading(start_heading_);
        motion_.fix_depth(start_depth_.get());
        motion_.thrust_forward(thrust_initial_search_.get(), timeout_looking_for_gate_.get());

        ROS_INFO_STREAM("Initialization has been completed. Working on heading: " << navig_.last_head()
            << ", depth: " << navig_.last_depth() << ", thrust: " << thrust_initial_search_.get() << std::endl);

        cmd_.set_recognizers(Camera::Front, {"fargate"});

        start_heading_ = navig_.last_head();

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

        if (is_gate_large()) {
            ++large_count_;
        } else {
            large_count_ = 0;
        }

        if (large_count_ >= large_count_needed_.get()) {
            return State::ProceedGate;
        }

        double gate_heading = lost_gate_ ? start_heading_ : get_new_head(center_);
        if (std::abs(gate_heading - start_heading_) >= heading_delta_.get()) {
            gate_heading = start_heading_;
        }

        ROS_INFO_STREAM("New heading: " << gate_heading);
        motion_.fix_heading(gate_heading);
        ROS_INFO_STREAM("fix_heading completed");
        motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_gate_.get());
        gate_found_ = false;

        return State::StabilizeGate;
    }

    State handle_proceed_gate()
    {
        if (proceed_started_) {
            return State::ProceedGate;
        }

        ROS_INFO_STREAM("Through the gate!" << "\n");
        double head = navig_.last_head();

        ROS_INFO_STREAM("Head to the gate = " << head << "\n");

        motion_.fix_heading(head, timeout_total_.get());
        motion_.thrust_forward(proceed_thrust_.get(), timeout_proceed_gate_.get());

        proceed_started_ = true;

        return State::ProceedGate;
    }

    void init_ipc(ipc::Communicator& comm)
    {
        sub_gate_ = comm.subscribe("vision", &GateTask::handle_gate_found, this);
    }

    void handle_gate_found(const vision::MsgFoundGate& msg)
    {
        if (msg.gate.empty()) {
            lost_gate_ = true;
            return;
        }

        lost_gate_ = false;

        auto left = msg.gate.front().left;
        auto right = msg.gate.front().right;

        x1_ = left.begin.y > left.end.y ? left.begin.x : left.end.x;
        x2_ = right.begin.y > right.end.y ? right.begin.x : right.end.x;

        auto x1 = front_camera_.frame_coord(MakePoint2(x1_, 0));
        auto x2 = front_camera_.frame_coord(MakePoint2(x2_, 0));
        center_ = (x1.x + x2.x) / 2;

        ROS_INFO_STREAM("Gate was found!" << std::endl);
        ROS_INFO_STREAM("Left leg: " << x1_ << ", right leg: " << x2_ << std::endl);
        ROS_INFO_STREAM("Center: " << center_ << std::endl);

        gate_found_ = true;
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_looking_for_gate_);
    AUTOPARAM(double, timeout_stabilize_gate_);
    AUTOPARAM(double, timeout_proceed_gate_);
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, heading_delta_);
    AUTOPARAM(double, thrust_initial_search_);
    AUTOPARAM(double, thrust_stabilize_);
    AUTOPARAM(double, proceed_thrust_);
    AUTOPARAM(double, gate_ratio_);
    AUTOPARAM(int, large_count_needed_);

    bool proceed_started_ = false;
    bool gate_found_ = false;
    int stabilize_count_ = 0;
    int x1_ = 0;
    int x2_ = 0;
    double center_ = 0.;
    double start_heading_ = 0;
    bool lost_gate_ = false;

    int large_count_ = 0;
    ipc::Subscriber<vision::MsgFoundGate> sub_gate_;

    bool is_gate_large()
    {
        if (!front_camera_.get_w()) {
            return false;
        }
        return front_camera_.get_w() ? (std::abs(x1_ - x2_) / front_camera_.get_w()) >= gate_ratio_.get() : false;
    }

    double get_new_head(double center)
    {
        double last_head = navig_.last_head();
        double angle = front_camera_.heading_to_point(MakePoint2(center, 0.));
        double new_head = R_to_DEG_ * normalize_angle(DEG_to_R_ * last_head + angle);

        ROS_INFO_STREAM("Last head = " << last_head << "\n");
        ROS_INFO_STREAM("Angle to object center = " << angle << "\n");
        ROS_INFO_STREAM("New head = " << new_head << "\n");

        return new_head;
    }
};

REGISTER_TASK(GateTask, gate_task);