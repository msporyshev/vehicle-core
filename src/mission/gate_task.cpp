#include "task.h"
#include "task_factory.h"

#include <video/MsgFoundGate.h>

#include <libauv/utils/math_u.h>

#include <algorithm>

enum class State
{
    Initialization,
    LookingForGate,
    StabilizeGate,
    ProceedGate,
    Terminal
};

class GateTask: public Task<State>
{
public:
    GateTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::LookingForGate)
    {
        state_machine_.REG_STATE(State::Initialization, handle_initialization, timeout_initialization_.get(), State::LookingForGate);
        state_machine_.REG_STATE(State::LookingForGate, handle_looking_for_gate, timeout_looking_for_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::StabilizeGate, handle_stabilize_gate, timeout_stabilize_gate_.get(), State::ProceedGate);
        state_machine_.REG_STATE(State::ProceedGate, handle_proceed_gate, timeout_proceed_gate_.get(), State::Terminal);

        init_ipc(comm);
    }

    State handle_initialization()
    {
        motion_.fix_pitch();
        motion_.fix_heading(start_heading_.is_set() ? start_heading_.get() : navig_.last_head());
        motion_.fix_depth(start_depth_.get());
        return State::LookingForGate;
    }

    State handle_looking_for_gate()
    {
        if (!initial_search_started_) {
            motion_.thrust_forward(thrust_initial_search_.get(), timeout_looking_for_gate_.get());
            initial_search_started_ = true;
        }
        return gate_found_ ? State::StabilizeGate : State::LookingForGate;
    }

    State handle_stabilize_gate()
    {
        bool is_stabilized = false;

        if(gate_found_) {
            if(stabilize()) stabilize_count_++;

            if(stabilize_count_ >= stabilize_count_needed_.get()) {
                ROS_INFO_STREAM("Stabilization success!" << "\n");
                is_stabilized = true;    
            }
            gate_found_ = false;
        }
        return is_stabilized ? State::ProceedGate : State::StabilizeGate;
    }

    State handle_proceed_gate()
    {
        ROS_INFO_STREAM("Through the gate!" << "\n");
        double head = navig_.last_head();

        ROS_INFO_STREAM("Head to the gate = " << head << "\n");

        motion_.fix_heading(head, timeout_total_.get());
        motion_.thrust_forward(proceed_thrust_.get(), timeout_proceed_gate_.get());

        return State::Terminal;
    }

    void init_ipc(ipc::Communicator& comm)
    {
        comm.subscribe("video", &GateTask::handle_gate_found, this);
    }

    void handle_gate_found(const video::MsgFoundGate& msg)
    {
        x1_ = (msg.gate.cols[0] + msg.gate.cols[1]) / 2;
        x2_ = (msg.gate.cols[2] + msg.gate.cols[3]) / 2;

        auto x1 = front_camera_.frame_coord(MakePoint2(x1_, 0));
        auto x2 = front_camera_.frame_coord(MakePoint2(x2_, 0));
        center_ = (x1.x + x2.x) / 2;

        gate_found_ = true;
    }

private:
    AUTOPARAM(double, timeout_initialization_);
    AUTOPARAM(double, timeout_looking_for_gate_);
    AUTOPARAM(double, timeout_stabilize_gate_);
    AUTOPARAM(double, timeout_proceed_gate_);
    AUTOPARAM_OPTIONAL(double, start_heading_, 0);
    AUTOPARAM(double, start_depth_);
    AUTOPARAM(double, thrust_initial_search_);
    AUTOPARAM(double, timeout_forward_);
    AUTOPARAM(int, stabilize_count_needed_);
    AUTOPARAM(double, proceed_thrust_);
    AUTOPARAM(double, kp_size_);
    AUTOPARAM(double, gate_size_);
    AUTOPARAM(double, desired_dist_);
    AUTOPARAM(double, eps_);

    bool gate_found_ = false;
    bool initial_search_started_ = false;
    int stabilize_count_ = 0;
    int x1_ = 0;
    int x2_ = 0;
    double center_ = 0.;

    bool stabilize()
    {
        double dist = front_camera_.calc_dist_to_object(gate_size_.get(), MakePoint2(x1_, 0), MakePoint2(x2_, 0));
        double head = get_new_head(center_);
        double thrust = get_new_thrust(dist, desired_dist_.get(), kp_size_.get());

        ROS_INFO_STREAM("Current dist to object = " << dist << "\n");
        ROS_INFO_STREAM("Heading to object = " << head << ", thrust = " << thrust << "\n");

        motion_.fix_heading(head, timeout_total_.get());
        motion_.thrust_forward(thrust, timeout_forward_.get());

        return std::abs(dist - desired_dist_.get()) <= eps_.get();
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

    double get_new_thrust(double current_dist_to_object, double desired_dist_to_object,
        double kp)
    {
        double err = current_dist_to_object - desired_dist_to_object;
        double stab_size_p = err * kp;

        if (stab_size_p > 0.15) {
            stab_size_p = 0.15;
        }

        ROS_INFO_STREAM("stab_size_p = " << stab_size_p << "\n");

        return stab_size_p;
    }
};

REGISTER_TASK(GateTask, gate_task);