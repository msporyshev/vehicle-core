#include "task.h"
#include "task_factory.h"

#include <mission/MsgRedBuoy.h>

#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
enum class State
{
    Init,
    BuoySearch,
    FixBuoy,
    FixBuoyDepth,
    KickBuoy,
    FinalMove,
    Terminal
};
}

class BuoyTask: public Task<State>
{
public:
    BuoyTask(const YamlReader& cfg, ipc::Communicator& comm): Task<State>(cfg, comm, State::Init)
    {
        state_machine_.REG_STATE(State::Init, handle_init,
            timeout_init_.get(), State::BuoySearch);

        state_machine_.REG_STATE(State::BuoySearch, handle_buoy_search,
            timeout_buoy_search_.get(), State::FixBuoy);

        state_machine_.REG_STATE(State::FixBuoy, handle_fix_buoy,
            timeout_fix_buoy_.get(), State::FixBuoyDepth);

        state_machine_.REG_STATE(State::FixBuoyDepth, handle_fix_buoy_depth,
            timeout_fix_buoy_depth_.get(), State::KickBuoy);

        state_machine_.REG_STATE(State::KickBuoy, handle_kick_buoy,
            timeout_kick_buoy_.get(), State::FinalMove);

        state_machine_.REG_STATE(State::FinalMove, handle_final_move,
            timeout_final_move_.get(), State::Terminal);


        channel_sub_ = comm.subscribe("mission", &BuoyTask::receive_buoy, this);
    }

    void receive_buoy(const mission::MsgRedBuoy& msg)
    {
        odometry_.add_frame_odometry(msg.odometry);
        current_buoy_ = msg;

        if (is_buoy_large()) {
            large_count_++;
        } else {
            large_count_ = 0;
        }

        buoy_found_ = true;
    }

    State handle_init()
    {
        motion_.thrust_forward(start_thrust_.get(), timeout_init_.get());

        cmd_.set_recognizers(Camera::Front, {"circle"});

        buoy_spot_heading_ = odometry_.head();

        return State::BuoySearch;
    }

    State handle_buoy_search()
    {
        return buoy_found_ ? State::FixBuoy : State::BuoySearch;
    }

    State handle_fix_buoy()
    {
        if (!buoy_found_) {
            return State::FixBuoy;
        }

        motion_.fix_heading(current_buoy_.pos.direction, WaitMode::DONT_WAIT);

        if (large_count_ >= large_count_needed_.get()) {
            motion_.fix_position(odometry_.frame_pos(), MoveMode::HEADING_FREE, timeout_fix_position_.get());
            buoy_spot_ = odometry_.frame_pos();
            return State::FixBuoyDepth;
        } else {
            motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_.get());
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

        motion_.fix_heading(current_buoy_.pos.direction, WaitMode::DONT_WAIT);
        motion_.fix_depth(current_buoy_.depth, WaitMode::DONT_WAIT);

        return State::FixBuoyDepth;
    }

    State handle_kick_buoy()
    {
        motion_.fix_heading(current_buoy_.pos.direction);
        motion_.move_forward(current_buoy_.pos.distance, timeout_move_forward_.get());
        motion_.fix_position(buoy_spot_, MoveMode::HOVER, timeout_move_forward_.get());

        return State::FinalMove;
    }

    State handle_final_move()
    {
        motion_.fix_heading(buoy_spot_heading_);
        motion_.move_forward(move_forward_distance_.get(), timeout_move_forward_.get());
        return State::Terminal;
    }

private:
    AUTOPARAM(double, start_thrust_);

    AUTOPARAM(double, timeout_init_);
    AUTOPARAM(double, timeout_buoy_search_);
    AUTOPARAM(double, timeout_fix_buoy_);
    AUTOPARAM(double, timeout_fix_buoy_depth_);
    AUTOPARAM(double, timeout_kick_buoy_);
    AUTOPARAM(double, timeout_final_move_);

    AUTOPARAM(int, large_count_needed_);
    AUTOPARAM(double, size_ratio_);
    AUTOPARAM(double, thrust_stabilize_);
    AUTOPARAM(double, timeout_stabilize_);

    AUTOPARAM(double, timeout_move_forward_);
    AUTOPARAM(double, move_forward_distance_);
    AUTOPARAM(double, timeout_fix_position_);

    navig::MsgLocalPosition buoy_spot_;
    double buoy_spot_heading_;
    mission::MsgRedBuoy current_buoy_;

    ipc::Subscriber<mission::MsgRedBuoy> channel_sub_;
    ros::Publisher gate_pub_;

    bool buoy_found_ = false;

    int large_count_ = 0;

    bool is_buoy_large()
    {
        return current_buoy_.size_ratio >= size_ratio_.get();
    }
};

REGISTER_TASK(BuoyTask, buoy_task);