/**
\file
\brief Реализация модуля определения координат акустических маяков

В данном файле находятся реализации методов, объявленных в dsp.h
Также здесь находится main

\defgroup dsp_node DSP
\brief Данный нод предназначен для инициализации соединения с DSP,
корректного выключения, а так же получения данных от DSP и их публикации в сеть.
*/

///@{
#include <cmath>

#include <dsp/MsgBeacon.h>
#include <dsp/MsgDebug.h>
#include <dsp/CmdSendCommand.h>
#include <utils/math_u.h>

#include "dsp.h"

const std::string Dsp::NODE_NAME = "dsp";

using namespace std;
using namespace utils;

Dsp::Dsp(ipc::Communicator& communicator) :
    communicator_(communicator), buffer_(nullptr)
{
    read_config();
    init_rx_buffer();
    init_ipc();

    max_delay_base_short_ = fabs(base_short_) * dsp_rate_ / sound_speed_ * 2.0;
    max_delay_base_long_ = fabs(base_long_) * dsp_rate_ / sound_speed_ * 2.0;

    if (connector_type_ == ConnectorType::UsbConnectorType) {
        con_ = new UsbConnector(buffer_, buffer_size_);
    }
    else {
        con_ = new ComConnector(com_name_, baudrate_, buffer_, buffer_size_);
    }


    dsp_preamble_ = 0x77EEFFC0;
    bearing_ = 0;
    distance_ = 0;
    depth_ = 0;
    north_ = 0;
    east_ = 0;
    heading_ = 0;
}

Dsp::~Dsp()
{
    delete[] buffer_;
    delete con_;
}

void Dsp::init_rx_buffer()
{
    if (buffer_) {
        delete[] buffer_;
        buffer_ = nullptr;
    }

    if(debug_mode_) {
        buffer_size_ = preamble_size_ + 3*(sizeof(short int)*1024) + 1;
    } else {
        buffer_size_ = preamble_size_ + 3*(sizeof(short int)) + 1;
    }

    buffer_ = new unsigned char[buffer_size_];
}

void Dsp::init_ipc()
{
    beacon_pub_ = communicator_.advertise<dsp::MsgBeacon>();
    debug_pub_ = communicator_.advertise<dsp::MsgDebug>();
    communicator_.subscribe_cmd<Dsp, dsp::CmdSendCommand>(&Dsp::handle_dsp_cmd, this);
    communicator_.subscribe("navig", &Dsp::handle_odometry, this);

}

void Dsp::read_config()
{
    ROS_ASSERT(ros::param::get("/dsp/base_short", base_short_));
    ROS_ASSERT(ros::param::get("/dsp/base_long", base_long_));
    ROS_ASSERT(ros::param::get("/dsp/sound_speed", sound_speed_));
    ROS_ASSERT(ros::param::get("/dsp/pinger_depth", pinger_depth_));
    ROS_ASSERT(ros::param::get("/dsp/default_pinger_frequency", default_pinger_frequency_));
    ROS_ASSERT(ros::param::get("/dsp/preamble_size", preamble_size_));
    ROS_ASSERT(ros::param::get("/dsp/connector_type", connector_type_str_));
    ROS_ASSERT(ros::param::get("/dsp/com_name", com_name_));
    ROS_ASSERT(ros::param::get("/dsp/baudrate", baudrate_));
    ROS_ASSERT(ros::param::get("/dsp/dsp_rate", dsp_rate_));
    ROS_ASSERT(ros::param::get("/dsp/channel_0", channel_0_));
    ROS_ASSERT(ros::param::get("/dsp/channel_1", channel_1_));
    ROS_ASSERT(ros::param::get("/dsp/channel_2", channel_2_));

    if(!ros::param::get("/dsp/debug_mode", debug_mode_)) {
        debug_mode_ = false;
    }

    if(connector_type_str_ == "com") {
        connector_type_ = ConnectorType::ComConnectorType;
    } else if(connector_type_str_ == "usb"){
        connector_type_ = ConnectorType::UsbConnectorType;
    } else {
        ROS_ASSERT_MSG(0, "FAIL: Unknown connector type, choose between \"com\" and \"usb\"");
    }

}

void Dsp::handle_odometry(const navig::MsgOdometry& msg)
{
    heading_ = msg.angle.heading;
    depth_ = msg.depth.distance;
    north_ = msg.pos.north;
    east_ = msg.pos.east;
}

void Dsp::set_mode(dsp::CommandType mode)
{
    con_->write_package(reinterpret_cast<unsigned char *>(&mode), 1);
}

int Dsp::package_processing()
{
    int preamble = *((int *)buffer_);

    beacon_type_ = *(reinterpret_cast<unsigned char *>(&buffer_[preamble_size_]));

    if(debug_mode_) {

        dsp::MsgDebug msg;

        vector<short> debug_data0(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1])), reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1 + sizeof(short)*1024])));
        copy(debug_data0.begin(), debug_data0.end(), msg.channel0.begin());

        vector<short> debug_data1(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1 + sizeof(short)*1024])), reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1 + sizeof(short)*2048])));
        copy(debug_data1.begin(), debug_data1.end(), msg.channel1.begin());

        vector<short> debug_data2(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1 + sizeof(short)*2048])), reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1 + sizeof(short)*3072])));
        copy(debug_data2.begin(), debug_data2.end(), msg.channel2.begin());

        debug_pub_.publish(msg);

    } else {

        short arrival_time[3] = {*(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 1]))),
                                 *(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 3]))),
                                 *(reinterpret_cast<short *>(&(buffer_[preamble_size_ + 5])))};

        if ((arrival_time[channel_0_] > 1024) || (arrival_time[channel_1_] > 1024) || (arrival_time[channel_2_] > 1024)) {
            ROS_DEBUG_STREAM("Pinger with id = " << beacon_type_ << " detected, couldn't calculate shifts between channels");
            return 0;
        }

        arrival_time[channel_1_] -= arrival_time[channel_0_];
        arrival_time[channel_2_] -= arrival_time[channel_0_];

        if ((arrival_time[channel_1_] < -max_delay_base_long_) || (arrival_time[channel_1_] > max_delay_base_long_))
        {
            ROS_DEBUG("Long base: time-of-arrival difference out of range!");
        }

        if ((arrival_time[channel_2_] < -max_delay_base_short_) || (arrival_time[channel_2_] > max_delay_base_short_))
        {
            ROS_DEBUG("Short base: time-of-arrival difference out of range!");
        }

        if ((arrival_time[channel_1_] < -max_delay_base_long_) || (arrival_time[channel_1_] > max_delay_base_long_) ||
           (arrival_time[channel_2_] < -max_delay_base_short_) || (arrival_time[channel_2_] > max_delay_base_short_))
        {
            ROS_DEBUG_STREAM("arrival_time1 = " << arrival_time[channel_1_] <<"(" << max_delay_base_long_ << ") arrival_time2 = " << arrival_time[channel_2_] <<"(" << max_delay_base_short_ << ")");
            return 0;
        }

        if ((arrival_time[channel_1_] == 0) && (arrival_time[channel_2_] == 0))
        {
            bearing_ = 0;

            distance_ = 0.0;

            ROS_DEBUG("Range = 0 (pinger is located under antenna!)");
        }
        else
        {
            // Расчет пеленга на пингер
            bearing_ = to_deg(atan2(arrival_time[channel_2_] * base_long_, arrival_time[channel_1_] * base_short_));

            // Расчет оценки сверху для горизонтальной дистанции

            // Квадрат продольного и поперечного смещения АНПА относительно пингера
            double dL2, dS2;

            // Разность хода лучей для длинной и короткой базы, м
            double dRL = fabs(arrival_time[channel_1_] * sound_speed_ / dsp_rate_);
            double dRS = fabs(arrival_time[channel_2_] * sound_speed_ / dsp_rate_);
            double dz = fabs(depth_ - pinger_depth_);

            if (dRL >= fabs(base_long_)) {
                dL2 = 10000.0;
            }
            else {
                dL2 = sqrt(dRL) * (0.25 + sqrt(dz) / (sqrt(base_long_) - sqrt(dRL)));
            }

            if (dRS >= fabs(base_short_)) {
                dS2 = 10000.0;
            }
            else {
                dS2 = sqrt(dRS) * (0.25 + sqrt(dz) / (sqrt(base_short_) - sqrt(dRS)));
            }

            distance_ = sqrt(dL2 + dS2);

            ROS_DEBUG_STREAM("Ping arrived, bearing: " << bearing_ << ", distance: " << distance_);
        }
    }

    return 1;
}

void Dsp::publish_beacon()
{
	dsp::MsgBeacon msg;

    msg.header.stamp = ros::Time::now();
	msg.bearing = bearing_;
    msg.distance = distance_;
    msg.beacon_type = beacon_type_;
    msg.heading = normalize_degree_angle(heading_ + bearing_);

    msg.north = cos(to_rad(msg.heading))*distance_ + north_;
    msg.east = sin(to_rad(msg.heading))*distance_ + east_;

    beacon_pub_.publish(msg);
}

void Dsp::handle_dsp_cmd(const dsp::CmdSendCommand& msg)
{
    if(msg.command < static_cast<unsigned char>(dsp::CommandType::Count)) {

        ROS_INFO_STREAM("Received command with id " << static_cast<int>(msg.command));

        if (msg.command == static_cast<unsigned char>(dsp::CommandType::DebugOn)) {
            debug_mode_ = true;
            init_rx_buffer();
        } else if (msg.command == static_cast<unsigned char>(dsp::CommandType::DebugOff)) {
            debug_mode_ = false;
            init_rx_buffer();
        }

        set_mode(static_cast<dsp::CommandType>(msg.command));
        con_->purge_handle();

    } else {
        ROS_ERROR("Invalid command received!");
    }
}

int main(int argc, char **argv)
{
	auto communicator = ipc::init(argc, argv, Dsp::NODE_NAME);
    Dsp dsp(communicator);

    if (dsp.con_->open() != 0) {
        ROS_ERROR("DSP_open failed");
        return -1;
    }

    ROS_INFO("Initialization was finished. DSP driver works");

    dsp.set_mode(dsp::CommandType::DspOff);

    if(dsp.debug_mode_) {
        dsp.set_mode(dsp::CommandType::DebugOn);
    } else {
        dsp.set_mode(dsp::CommandType::DebugOff);
    }

    dsp.set_mode(dsp::dsp_command.at(dsp.default_pinger_frequency_));
    dsp.set_mode(dsp::CommandType::DspOn);

    ipc::EventLoop loop(20);
    while(loop.ok())
    {
        if (dsp.con_->read_package() == 0) {
            if (dsp.package_processing()) {
                dsp.publish_beacon();
            }
        }
    }

    return 0;
}

///@}