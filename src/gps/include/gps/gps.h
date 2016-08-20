/**
\file
\brief Заголовочный файл драйвера GPS

В данном файле находится обьявление класса драйвера GPS

\ingroup gps_node
*/

///@{

#pragma once

#include <string>
#include <yaml_reader.h>

#include <libipc/ipc.h>
#include "ros/ros.h"

#include "gps/MsgGlobalPosition.h"
#include "gps/MsgSatellites.h"
#include "gps/MsgUtc.h"
#include "gps/MsgRaw.h"

typedef struct
{
  double UTC;
  char statusflag;
  double latitude;
  char ns;
  double longitude;
  char ew;
  int satellites;
  int CS;
} gps_data;

struct GpsConfig
{
    std::string port;
    int         baudrate;
    bool        simulating;

    GpsConfig() {
        port = "";
        baudrate = 0;
        simulating = false;
    }
};

class Gps
{
public:
    Gps(GpsConfig config);
    const static std::string NODE_NAME;

    int open_device();
    int close_device();

    void publish_data();
    void handle_gps_data(const ros::TimerEvent& event);
    void publish_sim_global_position(const ros::TimerEvent& event);
    void publish_sim_satellites(const ros::TimerEvent& event);
    void publish_sim_utc(const ros::TimerEvent& event);
    void publish_sim_raw(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);

private:
    int nmea_decode(std::vector<char> buffer);
    double get_utc_time(double gps_time);

    int device_descriptor_;
    GpsConfig config_;

    ros::Publisher  position_pub_,
                    satellites_pub_,
                    utc_pub_,
                    raw_pub_;
    
    bool msg_position_ready_;
    bool msg_sattelites_ready_;
    bool msg_utc_ready_;
    bool msg_raw_ready_;

    gps::MsgGlobalPosition msg_position_;
    gps::MsgSatellites msg_sattelites_;
    gps::MsgUtc msg_utc_;
    gps::MsgRaw msg_raw_;
};

///@}