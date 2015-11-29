#include "navig.h"

#include <navig/MsgNavigAccelerations.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>
#include <navig/MsgNavigVelocity.h>

using namespace std;

const string Navig::NODE_NAME = "navig";

Navig::Navig() :
    local_position_(make_pair(0., 0.)),
    longitude_(0.), latitude_(0.),
    acc_x_(0.), acc_y_(0.), acc_z_(0.),
    heading_(0.), pitch_(0.), roll_(0.),
    depth_(0.),
    distance_forward_(0.), distance_backward_(0.), distance_rightward_(0.), distance_leftward_(0.),
    height_(0.),
    rate_heading_(0.), rate_roll_(0.), rate_pitch_(0.),
    velocity_forward_(0.), velocity_right_(0.), velocity_down_(0.)
{}

Navig::Navig(const Navig& lhs) :
    local_position_(std::make_pair(lhs.local_position_.first, lhs.local_position_.second)),
    longitude_(lhs.longitude_), latitude_(lhs.latitude_),
    acc_x_(lhs.acc_x_), acc_y_(lhs.acc_y_), acc_z_(lhs.acc_z_),
    heading_(lhs.heading_), pitch_(lhs.pitch_), roll_(lhs.roll_),
    depth_(lhs.depth_),
    distance_forward_(lhs.distance_forward_), distance_backward_(lhs.distance_backward_), distance_rightward_(lhs.distance_rightward_), distance_leftward_(lhs.distance_leftward_),
    height_(lhs.height_),
    rate_heading_(lhs.rate_heading_), rate_roll_(lhs.rate_roll_), rate_pitch_(lhs.rate_pitch_),
    velocity_forward_(lhs.velocity_forward_), velocity_right_(lhs.velocity_right_), velocity_down_(lhs.velocity_down_)
{}

Navig::~Navig()
{}

void Navig::init_ipc(int argc, char* argv[], const string& node_name)
{
    auto communicator_ = ipc::init(argc, argv, node_name);
    acc_pub_ = communicator_.advertise<navig::MsgNavigAccelerations>("MsgNavigAcceleration");
    angles_pub_ = communicator_.advertise<navig::MsgNavigAngles>("MsgNavigAngles");
    depth_pub_ = communicator_.advertise<navig::MsgNavigDepth>("MsgNavigDepth");
    height_pub_ = communicator_.advertise<navig::MsgNavigHeight>("MsgNavigHeight"); 
    position_pub = communicator_.advertise<navig::MsgNavigPosition>("MsgNavigPosition"); 
    rates_pub_ = communicator_.advertise<navig::MsgNavigRates>("MsgNavigRates"); 
    velocity_pub_ = communicator_.advertise<navig::MsgNavigVelocity>("MsgNavigVelocity");
}

void Navig::create_and_publish_acc()
{
    navig::MsgNavigAccelerations msg;
    msg.acc_x = 10;
    msg.acc_y = 20;
    msg.acc_z = 3;
    acc_pub_.publish(msg);
}

void Navig::create_and_publish_angles()
{
    navig::MsgNavigAngles msg;
    msg.heading = 0;
    msg.roll = 90;
    msg.pitch = -90;
    angles_pub_.publish(msg);
}

void Navig::create_and_publish_depth()
{
    navig::MsgNavigDepth msg;
    msg.depth = 1.0;
    depth_pub_.publish(msg);
}

void Navig::create_and_publish_height()
{
    navig::MsgNavigHeight msg;
    msg.height = 1.0;
    height_pub_.publish(msg);
}

void Navig::create_and_publish_position()
{
    navig::MsgNavigPosition msg;
    msg.lon = 131;
    msg.lat = 43;
    msg.x = 1;
    msg.y = -1;
    position_pub.publish(msg);
}

void Navig::create_and_publish_rates()
{
    navig::MsgNavigRates msg;
    msg.rate_heading = 0.1;
    msg.rate_roll = 0.001;
    msg.rate_pitch = 0.001;
    rates_pub_.publish(msg);   
}

void Navig::create_and_publish_velocity()
{
    navig::MsgNavigVelocity msg;
    msg.velocity_forward = 1.0;
    msg.velocity_right = 0.5;
    msg.velocity_down = 0.1;
    velocity_pub_.publish(msg);
}

int main(int argc, char* argv[])
{
    Navig navig;
    navig.init_ipc(argc, argv, Navig::NODE_NAME);

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        navig.create_and_publish_acc();
        navig.create_and_publish_angles();
        navig.create_and_publish_depth();
        navig.create_and_publish_height();
        navig.create_and_publish_position();
        navig.create_and_publish_rates();
        navig.create_and_publish_velocity();
    }

    return 0;
}