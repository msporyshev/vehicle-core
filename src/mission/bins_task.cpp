#include "task.h"
#include "task_factory.h"

#include <utils/utils.h>

#include <vision/MsgFoundStripe.h>
#include <libipc/ipc.h>
#include <point/point.h>
#include <mission/MsgOrangeLane.h>
#include <camera_model.h>

namespace {

enum class State
{
    Init,
    LaneSearch,
    FixLane,
    OpenBin,
    Terminal,
};

}

using namespace utils;
using namespace std;

class BinsTask: public Task<State>
{
public:
    BinsTask(const YamlReader& cfg, ipc::Communicator& comm)
            : Task<State>(cfg, comm, State::Init)
    {
        state_machine_.REG_STATE(
            State::Init,
            handle_init,
            timeout_init_.get(),
            State::LaneSearch
            );

        state_machine_.REG_STATE(
            State::LaneSearch,
            handle_lane_search,
            timeout_lane_search_.get(),
            State::FixLane
            );


        state_machine_.REG_STATE(
            State::FixLane,
            handle_fix_lane,
            timeout_fix_lane_.get(),
            State::OpenBin
            );

        state_machine_.REG_STATE(
            State::OpenBin,
            handle_open_bin,
            timeout_open_bin_.get(),
            State::Terminal
            );

        stripe_sub_ = comm.subscribe("vision", &BinsTask::handle_found_stripe, this);
        lane_pub_ = comm.advertise<mission::MsgOrangeLane>();
    }

    void handle_found_stripe(const vision::MsgFoundStripe& msg) {
        odometry_.add_frame_odometry(msg.odometry);

        if (msg.stripes.empty()) {
            lanes_in_row_ = 0;
            return;
        }

        new_lane_ = true;

        lanes_in_row_++;

        ROS_INFO_STREAM("Lanes in a row: " << lanes_in_row_);

        auto target_stripe = msg.stripes.front();
        auto wbegin = bottom_camera_.frame_coord(target_stripe.wbegin);
        auto wend = bottom_camera_.frame_coord(target_stripe.wend);
        auto begin = bottom_camera_.frame_coord(target_stripe.begin);
        auto end = bottom_camera_.frame_coord(target_stripe.end);

        if (begin.y < end.y) {
            swap(begin, end);
        }

        current_lane_.center = (begin + end) * 0.5;

        double bearing = M_PI_2 - atan2(begin.y - end.y, begin.x - end.x);
        current_lane_.pos.bearing = to_deg(normalize_angle(bearing));
        current_lane_.pos.direction = normalize_degree_angle(
            odometry_.frame_head() + current_lane_.pos.bearing);

        current_lane_.pos.position =
            bottom_camera_.navig_offset_to_object(odometry_.frame_head(),
                real_lane_width_.get(), wbegin, wend, current_lane_.center);

        lane_pub_.publish(current_lane_);
    }

    State handle_init()
    {
        motion_.fix_pitch();
        ROS_INFO_STREAM("Fix current heading: " << odometry_.head());
        motion_.fix_heading(odometry_.head());

        ROS_INFO_STREAM("Fix depth: " << initial_depth_.get());
        motion_.fix_depth(initial_depth_.get());

        ROS_INFO_STREAM("Fix thrust: " << initial_thrust_.get());
        motion_.thrust_forward(initial_thrust_.get(), timeout_init_.get(), WaitMode::DONT_WAIT);

        cmd_.set_recognizers(Camera::Bottom, {"stripe"});
        return State::LaneSearch;
    }

    State handle_lane_search()
    {
        if (lanes_in_row_ < lanes_needed_.get()) {
            return State::LaneSearch;
        }

        return State::FixLane;
    }

    State handle_fix_lane()
    {
        if (lanes_in_row_ == 0) {
            return State::FixLane;
        }

        if (fix_by_position_.get()) {
            auto position = Point2d(current_lane_.pos.position.north, current_lane_.pos.position.east);
            motion_.fix_position(position, MoveMode::HEADING_FREE, timeout_position_.get(), WaitMode::DONT_WAIT);
        } else {
            Point2d fix_thrust_p = current_lane_.center * fix_p_.get();
            auto velocity = odometry_.frame_velocity();
            Point2d v = Point2d(velocity.right, velocity.forward);
            Point2d fix_thrust_d = v * (-fix_d_.get());
            Point2d fix_thrust_pd = fix_thrust_p + fix_thrust_d;

            // TODO разобраться с системами координат
            motion_.thrust_forward(fix_thrust_pd.y, timeout_regul_.get());
            motion_.thrust_right(fix_thrust_pd.x, timeout_regul_.get());
        }

        motion_.fix_heading(current_lane_.pos.direction, WaitMode::DONT_WAIT);
        if (norm(current_lane_.center) < eps_distance_.get()) {
            motion_.fix_position(odometry_.frame_pos(),
                MoveMode::HEADING_FREE, timeout_position_.get(), WaitMode::DONT_WAIT);
        }

        if (norm(current_lane_.center) < eps_distance_.get() && abs(current_lane_.pos.bearing) < eps_angle_.get()) {
            if (fix_start_time_ == 0) {
                fix_start_time_ = timestamp();
            }

            if (timestamp() - fix_start_time_ > success_time_.get()) {
                return State::OpenBin;
            }
        } else {
            fix_start_time_ = 0;
        }

        new_lane_ = false;

        return State::FixLane;
    }

    State handle_open_bin()
    {
        double start_depth = odometry_.depth().distance;
        ROS_INFO("Fix cur heading and move down");
        motion_.fix_heading(odometry_.head());
        motion_.thrust_backward(thrust_backward_.get(), timeout_backward_.get(), WaitMode::DONT_WAIT);
        motion_.move_down(bin_height_.get(), move_down_timeout_.get());

        ROS_INFO("Grabbing cover");

        double grab_depth = odometry_.depth().distance;
        cmd_.grab();
        motion_.move_up(distance_uncover_.get(), move_down_timeout_.get());
        ROS_INFO("Thrust backward");
        motion_.thrust_backward(thrust_uncover_.get(), timeout_uncover_.get(), WaitMode::WAIT);
        motion_.fix_depth(grab_depth, move_down_timeout_.get());
        ROS_INFO("Ungrab");
        cmd_.ungrab();

        motion_.thrust_forward(thrust_uncover_.get(), timeout_uncover_.get(), WaitMode::DONT_WAIT);
        motion_.fix_depth(start_depth);

        return State::Terminal;
    }

private:
    AUTOPARAM(double, initial_depth_);
    AUTOPARAM(double, initial_thrust_);

    AUTOPARAM(double, timeout_init_);
    AUTOPARAM(double, timeout_lane_search_);
    AUTOPARAM(double, timeout_fix_lane_);
    AUTOPARAM(double, timeout_open_bin_);

    AUTOPARAM(double, timeout_position_);
    AUTOPARAM(bool, fix_by_position_);
    AUTOPARAM(double, real_lane_width_);

    AUTOPARAM(double, fix_p_);
    AUTOPARAM(double, fix_d_);

    AUTOPARAM(double, eps_distance_);
    AUTOPARAM(double, eps_angle_);
    AUTOPARAM(double, success_time_);
    AUTOPARAM(int, lanes_needed_);

    AUTOPARAM(double, bin_height_);
    AUTOPARAM(double, move_down_timeout_);
    AUTOPARAM(double, thrust_backward_);
    AUTOPARAM(double, timeout_backward_);

    AUTOPARAM(double, distance_uncover_);
    AUTOPARAM(double, thrust_uncover_);
    AUTOPARAM(double, timeout_uncover_);

    int lanes_in_row_ = 0;
    double fix_start_time_ = 0;
    bool new_lane_ = false;

    mission::MsgOrangeLane current_lane_;

    ipc::Subscriber<vision::MsgFoundStripe> stripe_sub_;
    ros::Publisher lane_pub_;
};

REGISTER_TASK(BinsTask, bins_task);
