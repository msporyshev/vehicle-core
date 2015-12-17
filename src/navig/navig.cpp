#include "navig.h"

/**
\file
\brief Реализация навигационного модуля

В данном файле находятся реализации методов, объявленных в navig.h
Также здесь находится main

\defgroup navig_node Навигация
\brief Данный нод предназначен для реализации системы глобальной и локальной
навигации АНПА
*/

///@{
#include <navig/MsgNavigAccelerations.h>
#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigRates.h>
#include <navig/MsgNavigVelocity.h>

using namespace std;

const string Navig::NODE_NAME = "navig";

Navig::Navig(ipc::Communicator& communicator) :
    communicator_(communicator) 
{
    this->init_ipc();
}

Navig::~Navig()
{}

void Navig::init_ipc()
{    
    /**
        Это регистрация всех исходящих сообщений навига
    */
    acc_pub_ = communicator_.advertise<navig::MsgNavigAccelerations>();
    angles_pub_ = communicator_.advertise<navig::MsgNavigAngles>();
    depth_pub_ = communicator_.advertise<navig::MsgNavigDepth>();
    height_pub_ = communicator_.advertise<navig::MsgNavigHeight>(); 
    position_pub_ = communicator_.advertise<navig::MsgNavigPosition>(); 
    rates_pub_ = communicator_.advertise<navig::MsgNavigRates>(); 
    velocity_pub_ = communicator_.advertise<navig::MsgNavigVelocity>();

    /**
        Это подписка на сообщения
    */
    communicator_.subscribe("compass", &Navig::handle_angles, this);
    communicator_.subscribe("compass", &Navig::handle_acceleration, this);
    communicator_.subscribe("compass", &Navig::handle_rate, this);
    communicator_.subscribe("dvl", &Navig::handle_message<dvl::MsgDvlDistance>, this);
    communicator_.subscribe("dvl", &Navig::handle_message<dvl::MsgDvlVelocity>, this);
    communicator_.subscribe("dvl", &Navig::handle_message<dvl::MsgDvlHeight>, this);
    communicator_.subscribe("gps", &Navig::handle_message<gps::MsgGpsCoordinate>, this);
    communicator_.subscribe("gps", &Navig::handle_message<gps::MsgGpsSatellites>, this);
    communicator_.subscribe("gps", &Navig::handle_message<gps::MsgGpsUtc>, this);
    communicator_.subscribe("supervisor", &Navig::handle_message<supervisor::MsgSupervisorDepth>, this);
}

void Navig::handle_angles(const compass::MsgCompassAngle& msg)
{
    this->handle_message(msg);
    this->process_and_publish_angles(msg);
}

void Navig::handle_acceleration(const compass::MsgCompassAcceleration& msg)
{
    this->handle_message(msg);
    this->process_and_publish_acc(msg);
}

void Navig::handle_rate(const compass::MsgCompassAngleRate& msg)
{
    this->handle_message(msg);
    this->process_and_publish_rates(msg);
}

// void Navig::handle_distance_forward(const dvl::MsgDvlDistanceForward& msg) {}

// void Navig::handle_distance_leftward(const dvl::MsgDvlDistanceLeftward& msg) {}

// void Navig::handle_distance_rightward(const dvl::MsgDvlDistanceRightward& msg) {}

// void Navig::handle_velocity_down(const dvl::MsgDvlVelocityDown& msg) {}

// void Navig::handle_velocity_forward(const dvl::MsgDvlVelocityForward& msg) {}

// void Navig::handle_velocity_right(const dvl::MsgDvlVelocityRight& msg) {}

// void Navig::handle_height(const dvl::MsgDvlHeight& msg) {}

// void Navig::handle_coordinate(const gps::MsgGpsCoordinate& msg) {}

// void Navig::handle_satellites(const gps::MsgGpsSatellites& msg) {}

// void Navig::handle_utc(const gps::MsgGpsUtc& msg) {}

// void Navig::handle_depth(const sucan::MsgSucanDepth& msg) {}

void Navig::process_and_publish_acc(const compass::MsgCompassAcceleration& msg)
{
    navig::MsgNavigAccelerations nmsg;
    nmsg.acc_x = msg.acc_x;
    nmsg.acc_y = msg.acc_y;
    nmsg.acc_z = msg.acc_z;
    acc_pub_.publish(nmsg);
}

void Navig::process_and_publish_angles(const compass::MsgCompassAngle& msg)
{
    navig::MsgNavigAngles nmsg;
    nmsg.heading = msg.heading;
    nmsg.roll = msg.roll;
    nmsg.pitch = msg.pitch;
    angles_pub_.publish(nmsg);
}

void Navig::process_and_publish_rates(const compass::MsgCompassAngleRate& msg)
{
    navig::MsgNavigRates nmsg;
    nmsg.rate_heading = msg.rate_head;
    nmsg.rate_roll = msg.rate_roll;
    nmsg.rate_pitch = msg.rate_pitch;
    rates_pub_.publish(nmsg);   
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
    position_pub_.publish(msg);
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
    auto communicator = ipc::init(argc, argv, Navig::NODE_NAME);
    Navig navig(communicator);

    ipc::EventLoop loop(10);
    while(loop.ok()) {
        navig.create_and_publish_depth();
        navig.create_and_publish_height();
        navig.create_and_publish_position();
        navig.create_and_publish_velocity();
    }

    return 0;
}

///@}