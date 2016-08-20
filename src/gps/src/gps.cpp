/**
\file
\brief Реализация драйвера GPS

В данном файле находятся реализации методов, объявленных в gps.h

\ingroup gps_node
*/

///@{

#include <iostream>
#include <fstream>

#include "gps/gps.h"
#include "gps/NMEA_decode.h"
#include "gps/COM-ports.h"

#define LF 10
#define CR 13
#define DATA_OK 'A'

#define TIMEOUT 2.0

using namespace std;

const string Gps::NODE_NAME = "gps";

Gps::Gps(GpsConfig config): config_(config)
{
    device_descriptor_ = -1;

    msg_position_ready_ = false;
    msg_sattelites_ready_ = false;
    msg_utc_ready_ = false;
    msg_raw_ready_ = false;
}

void Gps::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    position_pub_      = comm.advertise<gps::MsgGlobalPosition>();
    satellites_pub_    = comm.advertise<gps::MsgSatellites>();
    utc_pub_           = comm.advertise<gps::MsgUtc>();
    raw_pub_           = comm.advertise<gps::MsgRaw>();
}

int Gps::open_device()
{
    char* port_name = const_cast<char*>(config_.port.c_str());
    int port_speed = config_.baudrate;

    device_descriptor_ = COM_open(port_name);

    if (device_descriptor_ == -1) {
        ROS_WARN_STREAM("Some problem with device opening. Check port name.");
        ROS_WARN_STREAM("Current port name: " << port_name);
        return -1;
    }

    if (COM_settings(device_descriptor_, port_speed) != 0) {
        ROS_WARN_STREAM("Some problem with port settings.");
        ROS_WARN_STREAM("Current port name: " << port_name);
        return -1;
    }

    ROS_DEBUG_STREAM("Device successfuly opened.");
    return 0;
}

int Gps::close_device()
{
    if(device_descriptor_ != -1) {
        ROS_INFO_STREAM("Device closed.");
        return COM_close(device_descriptor_);
    } else {
        ROS_WARN_STREAM("Try closing, but device was not open.");
    }
}

double Gps::get_utc_time(double gps_time)
{
    double utc_time;

    utc_time = fmod(gps_time, 100);         // Секунды и миллисекунды
    gps_time = floor(gps_time / 100);      // Часы и минуты
    utc_time += fmod(gps_time, 100) * 60;   // Учитываем минуты
    gps_time = floor(gps_time / 100);      // Часы
    utc_time += fmod(gps_time, 100) * 3600; // Учитываем часы

    return utc_time;
}

int Gps::nmea_decode(std::vector<char> buffer)
{
    int i = 0;
    char *field;
    int XOR;
    gps_data cur_data;

    char* ptr = buffer.data();

    XOR = Calc_NMEA_CS(ptr);

    if (XOR < 0){
        return -1;
    }

    if(GetField(ptr, &i, &field) != 0){
        return -1;
    }

    if(strcmp(field, "$GPGGA") == 0) {
        GetDouble(ptr, &i, &cur_data.UTC );
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetInt(ptr, &i, &cur_data.satellites);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetHex(ptr, &i, &cur_data.CS);

        if(cur_data.CS != XOR){
            return -1;
        } else {
              // Число задействованных спутников
            msg_sattelites_.satellites = (char)(cur_data.satellites % 0xFF);
            msg_raw_.satellites = (char)(cur_data.satellites % 0xFF);
            msg_sattelites_ready_ = true;
            
            msg_sattelites_.header.stamp = ros::Time::now();
            msg_raw_.header.stamp = ros::Time::now();
            
            // Единое (всемирное) время в формате double
            msg_raw_.utc = get_utc_time(cur_data.UTC);
            msg_utc_.utc = get_utc_time(cur_data.UTC);
            msg_utc_ready_ = true;
            msg_raw_ready_ = true;
        }

    } else if(strcmp(field, "$GPRMC") == 0){
        GetDouble(ptr, &i, &cur_data.UTC);
        GetChar(ptr, &i, &cur_data.statusflag);
        GetDouble(ptr, &i, &cur_data.latitude);
        GetChar(ptr, &i, &cur_data.ns);
        GetDouble(ptr, &i, &cur_data.longitude);
        GetChar(ptr, &i, &cur_data.ew);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetField(ptr, &i, &field);
        GetHex(ptr, &i, &cur_data.CS);

        if(cur_data.CS != XOR){
            return -1;
        }

        bool good_data = cur_data.statusflag == DATA_OK ? true : false;

        double lat, lon;
        lat = cur_data.latitude / 100;
        lat = (floor(lat) + fmod(lat, 1) / 0.60);
        if(cur_data.ns == 's'){
            lat = -lat;
        }

        lon = cur_data.longitude / 100;
        lon = (floor(lon) + fmod(lon, 1) / 0.60);
        if ( cur_data.ew == 'w' ) {
            lon = -lon;
        }

        msg_raw_.latitude = lat;
        msg_raw_.longitude = lon;
        msg_raw_ready_ = true;
        msg_raw_.header.stamp = ros::Time::now();

        if(good_data) {
            msg_position_.latitude = lat;
            msg_position_.longitude = lon;
            msg_position_ready_ = true;
            msg_position_.header.stamp = ros::Time::now();
        }
        // Единое (всемирное) время в формате double
        msg_raw_.utc = get_utc_time(cur_data.UTC);
        msg_utc_.utc = get_utc_time(cur_data.UTC);
        msg_utc_.header.stamp = ros::Time::now();
        msg_utc_ready_ = true;
        msg_raw_ready_ = true;
  }else{
    return -1;
  }

  return 0;
}

void Gps::publish_data()
{
    if(msg_raw_ready_) {
        raw_pub_.publish(msg_raw_);
    }

    if(msg_sattelites_ready_) {
        satellites_pub_.publish(msg_position_);
        msg_sattelites_ready_ = false;
    }
    if(msg_utc_ready_) {
        utc_pub_.publish(msg_sattelites_);
        msg_utc_ready_ = false;
    }
    if(msg_position_ready_) {
        position_pub_.publish(msg_utc_);
        msg_position_ready_ = false;
    }
}

void Gps::handle_gps_data(const ros::TimerEvent& event)
{
    char c;
    std::vector<char> buffer;
    bool start_found = false;

    ROS_ASSERT_MSG(device_descriptor_ != -1, "Port was not configure. Emergency exit.");
    
    ros::Time start_time = ros::Time::now();

    while(ros::Time::now() > start_time + ros::Duration(TIMEOUT)){
        int read_count = read(device_descriptor_, &c, 1);
        if(read_count < 1){
            ROS_DEBUG_STREAM("Nothing to read, sleep 50 msec.");
            ros::Duration(0.05).sleep();
            continue;
        }

        if(c == '$'){
            start_found = true;
            ROS_DEBUG_STREAM("Found $ simbol.");
        } else if(start_found){
            buffer.push_back(c);
        }

        if(start_found && (*buffer.end() - 1) == LF && (*buffer.end() - 2) == CR){
            ROS_DEBUG_STREAM("NMEA received, try to decoding.");
            int decode_count = nmea_decode(buffer);
            if(decode_count == 0){
                publish_data();
            } else {
                ROS_ERROR_STREAM("Decoding failed.");
            }
            return;
        }
    }

    ROS_ERROR_STREAM("Returning from handle_gps_data by timeout.");
    return;
}

void Gps::publish_sim_global_position(const ros::TimerEvent& event)
{
    static double lat = 135, lon = 54;

    lat = lat < 137 ? lat + 0.001 : 135;
    lon = lon < 56 ? lon + 0.001 : 54;

    gps::MsgGlobalPosition msg;
    msg.latitude = lat;
    msg.longitude = lon;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    position_pub_.publish(msg);
}

void Gps::publish_sim_satellites(const ros::TimerEvent& event)
{
    static double sat = 0;
    sat = sat < 16 ? sat + 1 : 0;

    gps::MsgSatellites msg;
    msg.satellites = sat;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    satellites_pub_.publish(msg);
}

void Gps::publish_sim_utc(const ros::TimerEvent& event)
{
    gps::MsgUtc msg;
    msg.utc = ros::Time::now().toSec();
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    utc_pub_.publish(msg);
}

void Gps::publish_sim_raw(const ros::TimerEvent& event)
{
    static double lat = 135, lon = 54;

    lat = lat < 137 ? lat + 0.001 : 135;
    lon = lon < 56 ? lon + 0.001 : 54;

    static double sat = 0;
    sat = sat < 16 ? sat + 1 : 0;

    gps::MsgRaw msg;
    msg.utc = ros::Time::now().toSec();
    msg.satellites = sat;
    msg.latitude = lat;
    msg.longitude = lon;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg));
    raw_pub_.publish(msg);
}
///@}