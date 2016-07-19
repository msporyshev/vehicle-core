#include <utils/node_utils.h>
#include <utils/utils.h>

#include <memory>

#include <libipc/ipc.h>

#include <mission/MsgOrangeLane.h>
#include <mission/MsgValidationGate.h>
#include <mission/MsgNavigateChannel.h>
#include <mission/MsgRedBuoy.h>

#include <vision/MsgFoundStripe.h>
#include <vision/MsgFoundGate.h>
#include <vision/MsgFoundCircle.h>
#include <vision/MsgFoundBin.h>

#include <dsp/MsgBeacon.h>
#include <mission/MsgPingerPosition.h>

#include <camera_model/camera_model.h>
#include <config_reader/yaml_reader.h>
#include "odometry.h"

using namespace vision;
using namespace mission;
using namespace navig;
using namespace std;
using namespace utils;

ros::Publisher orange_lane_pub;
ros::Publisher validation_gate_pub;
ros::Publisher navigate_channel_pub;
ros::Publisher red_buoy_pub;
ros::Publisher pinger_position_pub;

shared_ptr<Odometry> odometry;
FrontCamera front_camera;
BottomCamera bottom_camera;

dsp::MsgBeacon current_dsp_msg_;
int is_bearing_received_ = 0;
double trusted_coef_ = 0.1;

MsgObjectPosition calc_object_position(
        const CameraModel& camera,
        const MsgOdometry odometry,
        double real_size,
        Point2d start,
        Point2d end,
        Point2d center
        )
{
    MsgObjectPosition pos;

    pos.position = odometry.pos + camera.navig_offset_to_object(
        odometry.angle.heading, real_size, start, end, center);
    pos.bearing = camera.bearing_to_point(center);
    pos.direction = normalize_degree_angle(pos.bearing + odometry.angle.heading);
    pos.distance = camera.calc_dist_to_object(real_size, start, end);

    return pos;
}

MsgObjectPosition calc_object_position(
        const CameraModel& camera,
        const MsgOdometry odometry,
        double real_size,
        int pixel_size,
        Point2d pixel
        )
{
    MsgObjectPosition pos;

    pos.position = odometry.pos + camera.navig_offset_to_object(
        odometry.angle.heading, real_size, pixel_size, pixel);
    pos.bearing = camera.bearing_to_point(camera.frame_coord(pixel));
    pos.direction = normalize_degree_angle(pos.bearing + odometry.angle.heading);
    pos.distance = camera.calc_dist_to_object(real_size, pixel_size);

    return pos;
}

void receive_bins(const MsgFoundBin& msg)
{
    if (msg.bins.empty()) {
        return;
    }
}

void receive_buoy(const MsgFoundCircle& msg)
{
    if (msg.circles.empty()) {
        return;
    }

    MsgCircle red_circle;
    for (auto& circle : msg.circles) {
        if (circle.radius > red_circle.radius) {
            red_circle = circle;
        }
    }

    YamlReader cfg("buoy_task.yml", "mission");
    double buoy_real_size = cfg.read_as<double>("buoy_real_size");

    MsgRedBuoy current_buoy;

    current_buoy.pixel_size = red_circle.radius;
    current_buoy.size_ratio = red_circle.radius / front_camera.get_w();

    current_buoy.center = front_camera.frame_coord(red_circle.center);

    current_buoy.pos = calc_object_position(front_camera, msg.odometry,
        buoy_real_size, red_circle.radius, red_circle.center);
    current_buoy.depth = msg.odometry.depth.distance
        + front_camera.calc_depth_to_object(buoy_real_size, red_circle.radius, red_circle.center);

    current_buoy.odometry = msg.odometry;
}

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
    auto lwidth = msg.gate.front().left.width;
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

    current_channel.left_x = p1.x;
    current_channel.left_width_ratio = lwidth / front_camera.get_w();

    current_channel.left_bottom = lend.y;
    current_channel.right_bottom = rend.y;
    current_channel.width_ratio = std::abs(p2.x - p2.x) / front_camera.get_w();

    current_channel.direction_left =
        normalize_degree_angle(odometry->frame_head() + front_camera.bearing_to_point(p1));

    current_channel.center = (p1 + p2) * 0.5;
    current_channel.pos = calc_object_position(front_camera, msg.odometry,
        gate_real_size, p1, p2, current_channel.center);

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

    current_lane.pos = calc_object_position(bottom_camera, msg.odometry, real_lane_width,
        wbegin, wend, current_lane.center);
    double bearing = M_PI_2 - atan2(begin.y - end.y, begin.x - end.x);
    current_lane.pos.bearing = utils::to_deg(normalize_angle(bearing));
    current_lane.pos.direction = normalize_degree_angle(msg.odometry.angle.heading + current_lane.pos.bearing);

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

    current_gate.pos = calc_object_position(front_camera, msg.odometry,
        gate_real_size, p1, p2, current_gate.center);

    current_gate.odometry = msg.odometry;

    validation_gate_pub.publish(current_gate);
}

void receive_pinger_bearing(const dsp::MsgBeacon& msg)
{
    current_dsp_msg_ = msg;
    is_bearing_received_ = 1;
}

void calculate_pinger_coordinates(const ros::TimerEvent&)
{
    static double weight_old = 0;
    static double weight_north_old = 0;
    static double weight_east_old = 0;

    double north = odometry->pos().north;
    double east  = odometry->pos().east;

    double pinger_north = north + current_dsp_msg_.distance * cos(to_rad(current_dsp_msg_.heading));
    double pinger_east  = east + current_dsp_msg_.distance * sin(to_rad(current_dsp_msg_.heading));

    double weight       = weight_old * (1 - trusted_coef_) + trusted_coef_ * is_bearing_received_;
    double weight_north = weight_north_old * (1 - trusted_coef_) + pinger_north * trusted_coef_ * is_bearing_received_;
    double weight_east  = weight_east_old * (1 - trusted_coef_) + pinger_east * trusted_coef_ * is_bearing_received_;

    weight_old       = weight;
    weight_north_old = weight_north;
    weight_east_old  = weight_east;

    if(is_bearing_received_) {
        mission::MsgPingerPosition msg;

        msg.position.north = weight_north / weight;
        msg.position.east = weight_east / weight;

        pinger_position_pub.publish(msg);
    }

    is_bearing_received_ = 0;
}

int main(int argc, char* argv[])
{
    auto comm = ipc::init(argc, argv, "visual_nav");

    odometry = make_shared<Odometry>(comm);

    orange_lane_pub = comm.advertise_cmd<MsgOrangeLane>("mission");
    validation_gate_pub = comm.advertise_cmd<MsgValidationGate>("mission");
    navigate_channel_pub = comm.advertise_cmd<MsgNavigateChannel>("mission");
    red_buoy_pub = comm.advertise_cmd<MsgRedBuoy>("mission");

    pinger_position_pub = comm.advertise_cmd<MsgPingerPosition>("mission");

    auto orange_stripe_sub = comm.subscribe("vision", receive_orange_stripe);
    auto validation_gate_sub = comm.subscribe("vision", receive_validation_gate);
    auto navigate_channel_sub = comm.subscribe("vision", receive_navigate_channel);
    auto buoy_sub = comm.subscribe("vision", receive_buoy);

    auto pinger_sub = comm.subscribe("dsp", receive_pinger_bearing);

    auto timer_calculation_ = comm.create_timer(1, &calculate_pinger_coordinates);

    ros::spin();
}