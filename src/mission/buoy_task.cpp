#include "task.h"
#include "task_factory.h"

#include <mission/MsgRedBuoy.h>
#include <mission/MsgGreenBuoy.h>
#include <mission/MsgBuoy.h>

#include <utils/math_u.h>

#include <algorithm>

using namespace utils;

namespace {
enum class State
{
    Init,
    BuoySearch,
    FixBuoySpot,
    SelectBuoy,
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
            timeout_buoy_search_.get(), State::SelectBuoy);

        state_machine_.REG_STATE(State::FixBuoySpot, handle_fix_buoy_spot,
            timeout_fix_buoy_spot_.get(), State::SelectBuoy);

        state_machine_.REG_STATE(State::SelectBuoy, handle_select_buoy,
            timeout_select_buoy_.get(), State::FixBuoy);

        state_machine_.REG_STATE(State::FixBuoy, handle_fix_buoy,
            timeout_fix_buoy_.get(), State::FixBuoyDepth);

        state_machine_.REG_STATE(State::FixBuoyDepth, handle_fix_buoy_depth,
            timeout_fix_buoy_depth_.get(), State::KickBuoy);

        state_machine_.REG_STATE(State::KickBuoy, handle_kick_buoy,
            timeout_kick_buoy_.get(), State::FinalMove);

        state_machine_.REG_STATE(State::FinalMove, handle_final_move,
            timeout_final_move_.get(), State::Terminal);


        buoy_sub_ = comm.subscribe("mission", &BuoyTask::receive_buoy, this, 5);
        green_buoy_pub_ = comm.advertise<mission::MsgGreenBuoy>();
        red_buoy_pub_ = comm.advertise<mission::MsgGreenBuoy>();
    }

    void receive_buoy(const mission::MsgBuoy& msg)
    {
        odometry_.add_frame_odometry(msg.odometry);
        current_buoy_ = msg;

        if (is_buoy_large()) {
            large_count_++;
        } else {
            large_count_ = 0;
        }

        Color color = static_cast<Color>(msg.color);
        if (color == Color::Red) {
            red_buoy_ = msg;
            red_count_++;

            mission::MsgRedBuoy rb;
            rb.pos = msg.pos;
            red_buoy_pub_.publish(rb);
        } else if (color == Color::Green) {
            green_buoy_ = msg;
            green_count_++;

            mission::MsgGreenBuoy gb;
            gb.pos = msg.pos;
            green_buoy_pub_.publish(gb);
        }

        if (kick_red_) {
            current_buoy_ = red_buoy_;
        } else if (kick_green_) {
            current_buoy_ = green_buoy_;
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
        if (green_count_ && red_count_) {
            return State::FixBuoySpot;
        } else {
            return State::BuoySearch;
        }
    }

    State handle_fix_buoy_spot()
    {
        if (!buoy_found_) {
            return State::FixBuoySpot;
        }
        buoy_found_ = false;

        if (red_count_ && green_count_) {
            double bearing = (red_buoy_.pos.bearing + green_buoy_.pos.bearing) * 0.5;
            buoy_spot_heading_ = normalize_degree_angle(odometry_.frame_head() + bearing);
            ROS_INFO_STREAM("Buoy spot heading: " << buoy_spot_heading_);

            if (are_buoys_close()) {
                buoy_spot_ = odometry_.frame_pos();
                motion_.fix_position(buoy_spot_, MoveMode::HEADING_FREE, timeout_fix_position_.get());
            } else {
                motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_.get());
            }
        } else {
            motion_.thrust_forward(thrust_stabilize_.get(), timeout_stabilize_.get());
        }

        if (red_count_ > count_needed_.get() && green_count_ > count_needed_.get() && are_buoys_close()) {
            return State::SelectBuoy;
        }

        motion_.fix_heading(buoy_spot_heading_, WaitMode::DONT_WAIT);

        return State::FixBuoySpot;
    }

    State handle_select_buoy()
    {
        if (red_count_ && !kick_red_) {
            kick_red_ = true;
            motion_.fix_heading(red_buoy_.pos.direction);
            current_buoy_ = red_buoy_;
        } else if (green_count_ && !kick_green_) {
            kick_green_ = true;
            motion_.fix_heading(green_buoy_.pos.direction);
            current_buoy_ = green_buoy_;
        } else {
            return State::FinalMove;
        }

        return State::FixBuoy;
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

        if (kick_green_ && kick_red_) {
            return State::FinalMove;
        } else {
            green_count_ = 0;
            red_count_ = 0;
            return State::FixBuoySpot;
        }
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
    AUTOPARAM(double, timeout_fix_buoy_spot_);
    AUTOPARAM(double, timeout_select_buoy_);
    AUTOPARAM(double, timeout_fix_buoy_);
    AUTOPARAM(double, timeout_fix_buoy_depth_);
    AUTOPARAM(double, timeout_kick_buoy_);
    AUTOPARAM(double, timeout_final_move_);

    AUTOPARAM(double, dist_needed_);
    AUTOPARAM(double, count_needed_);

    AUTOPARAM(int, large_count_needed_);
    AUTOPARAM(double, size_ratio_);
    AUTOPARAM(double, thrust_stabilize_);
    AUTOPARAM(double, timeout_stabilize_);

    AUTOPARAM(double, timeout_move_forward_);
    AUTOPARAM(double, move_forward_distance_);
    AUTOPARAM(double, timeout_fix_position_);

    navig::MsgLocalPosition buoy_spot_;
    double buoy_spot_heading_;

    mission::MsgBuoy current_buoy_;
    mission::MsgBuoy red_buoy_;
    mission::MsgBuoy green_buoy_;

    ipc::Subscriber<mission::MsgBuoy> buoy_sub_;
    ros::Publisher red_buoy_pub_;
    ros::Publisher green_buoy_pub_;

    bool buoy_found_ = false;

    bool kick_green_ = false;
    bool kick_red_ = false;

    int large_count_ = 0;
    int red_count_ = 0;
    int green_count_ = 0;

    bool are_buoys_close()
    {
        return abs(red_buoy_.center.x - green_buoy_.center.x) >= dist_needed_.get();
    }

    bool is_buoy_large()
    {
        return current_buoy_.size_ratio >= size_ratio_.get();
    }
};

REGISTER_TASK(BuoyTask, buoy_task);