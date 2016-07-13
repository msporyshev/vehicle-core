#include <utils/node_utils.h>
#include <utils/utils.h>

#include <memory>

#include <libipc/ipc.h>

#include <mission/MsgOrangeLane.h>
#include <mission/MsgValidationGate.h>
#include <mission/MsgNavigateChannel.h>
#include <vision/MsgFoundStripe.h>
#include <vision/MsgFoundGate.h>

#include <camera_model/camera_model.h>
#include <config_reader/yaml_reader.h>
#include "odometry.h"

using namespace vision;
using namespace mission;
using namespace std;
using namespace utils;

ros::Publisher orange_lane_pub;
ros::Publisher validation_gate_pub;
ros::Publisher navigate_channel_pub;

shared_ptr<Odometry> odometry;
CameraModel front_camera = CameraModel::create_front_camera();
CameraModel bottom_camera = CameraModel::create_bottom_camera();

void receive_navigate_channel(const MsgFoundGate& msg)
{
    if (msg.gate.empty()) {
        return;
    }

    YamlReader cfg("navigate_channel_task.yml", "mission");
    double gate_real_size = cfg.read_as<double>("channel_real_size");

    MsgNavigateChannel current_channel;

    odometry->add_frame_odometry(msg.odometry);


    auto left = msg.gate.front().left;
    auto right = msg.gate.front().right;

    auto lbegin = front_camera.frame_coord(left.begin);
    auto lend = front_camera.frame_coord(left.end);
    if (lbegin.y < lend.y) {
        swap(lbegin, lend);
    }

    auto rbegin = front_camera.frame_coord(right.begin);
    auto rend = front_camera.frame_coord(right.end);
    if (rbegin.y < rend.y) {
        swap(rbegin, rend);
    }

    auto p1 = (lbegin + lend) * 0.5;
    auto p2 = (rbegin + rend) * 0.5;

    double center = (p1.x + p2.x) / 2;

    current_channel.frame_number = msg.frame_number;

    current_channel.left_bottom = lend.y;
    current_channel.right_bottom = rend.y;
    current_channel.width_ratio = std::abs(p2.x - p2.x) / front_camera.get_w();

    current_channel.center = (p1 + p2) * 0.5;
    double heading_delta = front_camera.heading_to_point(current_channel.center);
    current_channel.bearing = heading_delta;
    current_channel.direction =
        normalize_degree_angle(heading_delta + odometry->frame_head());

    current_channel.distance = front_camera.calc_dist_to_object(gate_real_size, p1, p2);
    current_channel.position = odometry->front_target_pos(gate_real_size, p1, p2, current_channel.center);

    current_channel.odometry = msg.odometry;

    navigate_channel_pub.publish(current_channel);
}

void receive_orange_stripe(const MsgFoundStripe& msg)
{
    odometry->add_frame_odometry(msg.odometry);

    if (msg.stripes.empty()) {
        return;
    }

    YamlReader cfg("orange_lane_task.yml", "mission");
    double real_lane_width = cfg.read_as<double>("real_lane_width");

    auto target_stripe = msg.stripes.front();
    auto wbegin = bottom_camera.frame_coord(target_stripe.wbegin);
    auto wend = bottom_camera.frame_coord(target_stripe.wend);
    auto begin = bottom_camera.frame_coord(target_stripe.begin);
    auto end = bottom_camera.frame_coord(target_stripe.end);

    if (begin.y < end.y) {
        swap(begin, end);
    }

    MsgOrangeLane current_lane;
    current_lane.center = (begin + end) * 0.5;

    double bearing = M_PI_2 - atan2(begin.y - end.y, begin.x - end.x);
    current_lane.bearing = to_deg(normalize_angle(bearing));
    current_lane.direction = normalize_degree_angle(
        odometry->frame_head() + current_lane.bearing);

    double frame_width = norm(wbegin - wend);
    current_lane.position =
        odometry->bottom_target_pos(real_lane_width, frame_width, current_lane.center);

    current_lane.odometry = msg.odometry;

    orange_lane_pub.publish(current_lane);
}

// Куда девать конфигурационные данные (параметры объекта)?
// Миссия или зрение? или отдельный модуль?
void receive_validation_gate(const MsgFoundGate& msg)
{
    if (msg.gate.empty()) {
        return;
    }

    YamlReader cfg("gate_task.yml", "mission");
    double gate_real_size = cfg.read_as<double>("gate_real_size");

    MsgValidationGate current_gate;

    odometry->add_frame_odometry(msg.odometry);


    auto left = msg.gate.front().left;
    auto right = msg.gate.front().right;

    double x1_ = left.begin.x;
    double x2_ = right.begin.x;

    auto p1 = front_camera.frame_coord(Point2d(x1_, 0));
    auto p2 = front_camera.frame_coord(Point2d(x2_, 0));

    double center = (p1.x + p2.x) / 2;

    current_gate.frame_number = msg.frame_number;

    current_gate.width_ratio = std::abs(x2_ - x1_) / front_camera.get_w();

    current_gate.center = (p1 + p2) * 0.5;
    double heading_delta = front_camera.heading_to_point(current_gate.center);
    current_gate.bearing = heading_delta;
    current_gate.direction =
        normalize_degree_angle(heading_delta + odometry->frame_head());

    current_gate.distance = front_camera.calc_dist_to_object(gate_real_size, p1, p2);
    current_gate.position = odometry->front_target_pos(gate_real_size, p1, p2, current_gate.center);

    current_gate.odometry = msg.odometry;

    validation_gate_pub.publish(current_gate);
}

int main(int argc, char* argv[])
{
    auto comm = ipc::init(argc, argv, "visual_nav");

    odometry = make_shared<Odometry>(comm);

    orange_lane_pub = comm.advertise_cmd<MsgOrangeLane>("mission");
    validation_gate_pub = comm.advertise_cmd<MsgValidationGate>("mission");

    auto orange_stripe_sub = comm.subscribe("vision", receive_orange_stripe);
    auto validation_gate_sub = comm.subscribe("vision", receive_validation_gate);

    ros::spin();
}