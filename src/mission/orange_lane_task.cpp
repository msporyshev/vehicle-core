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
    Terminal,
};

}

using namespace utils;
using namespace std;

class OrangeLaneTask: public Task<State>
{
public:
    OrangeLaneTask(const YamlReader& cfg, ipc::Communicator& comm)
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
            State::Terminal
            );

        stripe_sub_ = comm.subscribe("vision", &OrangeLaneTask::handle_found_stripe, this);
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
        current_lane_.bearing = to_deg(normalize_angle(bearing));
        current_lane_.direction = normalize_degree_angle(
            odometry_.frame_head() + current_lane_.bearing);

        double frame_width = norm(wbegin - wend);
        current_lane_.position =
            odometry_.bottom_target_pos(real_lane_width_.get(), frame_width, current_lane_.center);

        lane_pub_.publish(current_lane_);
    }

    State handle_init()
    {
        // motion_.fix_pitch();
        ROS_INFO_STREAM("Fix current heading: " << odometry_.head());
        motion_.fix_heading(odometry_.head());

        ROS_INFO_STREAM("Fix depth: " << initial_depth_.get());
        motion_.fix_depth(initial_depth_.get());

        ROS_INFO_STREAM("Fix thrust: " << initial_thrust_.get());
        motion_.thrust_forward(initial_thrust_.get(), timeout_init_.get());

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
            auto position = Point2d(current_lane_.position.north, current_lane_.position.east);
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

        motion_.fix_heading(current_lane_.direction, WaitMode::DONT_WAIT);
        if (norm(current_lane_.center) < eps_distance_.get()) {
            motion_.fix_position(odometry_.frame_pos(),
                MoveMode::HEADING_FREE, timeout_position_.get(), WaitMode::DONT_WAIT);
        }

        if (norm(current_lane_.center) < eps_distance_.get() && abs(current_lane_.bearing) < eps_angle_.get()) {
            if (fix_start_time_ == 0) {
                fix_start_time_ = timestamp();
            }

            if (timestamp() - fix_start_time_ > success_time_.get()) {
                return State::Terminal;
            }
        } else {
            fix_start_time_ = 0;
        }

        new_lane_ = false;

        return State::FixLane;
    }

private:
    AUTOPARAM(double, initial_depth_);
    AUTOPARAM(double, initial_thrust_);
    AUTOPARAM(double, timeout_init_);
    AUTOPARAM(double, timeout_lane_search_);
    AUTOPARAM(double, timeout_fix_lane_);

    AUTOPARAM(double, timeout_position_);
    AUTOPARAM(bool, fix_by_position_);
    AUTOPARAM(double, real_lane_width_);

    AUTOPARAM(double, fix_p_);
    AUTOPARAM(double, fix_d_);

    AUTOPARAM(double, eps_distance_);
    AUTOPARAM(double, eps_angle_);
    AUTOPARAM(double, success_time_);
    AUTOPARAM(int, lanes_needed_);

    int lanes_in_row_ = 0;
    double fix_start_time_ = 0;
    bool new_lane_ = false;

    mission::MsgOrangeLane current_lane_;

    ipc::Subscriber<vision::MsgFoundStripe> stripe_sub_;
    ros::Publisher lane_pub_;
};

REGISTER_TASK(OrangeLaneTask, orange_lane_task);
