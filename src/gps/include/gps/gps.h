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
#include "gps/MsgCorrections.h"
#include "gps/MsgHeight.h"
#include "gps/MsgMagnDeclination.h"
#include "gps/MsgMotion.h"
#include "gps/MsgStatus.h"

#include "sentence.h"
#include "info.h"
#include "parser.h"
#include "nmath.h"

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
    void init_connection(ipc::Communicator& communicator);

private:
    void publish_gprms_data();
    void publish_gpgga_data();

    bool get_data(int dd, std::vector<char>& buffer);
    bool nmea_decode(std::vector<char> buffer);

    void handle_gps_data(const ros::TimerEvent& event);
    void simulate_gps_data(const ros::TimerEvent& event);
    void publish_status(const ros::TimerEvent& event);

    int device_descriptor_;
    GpsConfig config_;
    
    NmeaInfo        info_;
    NmeaParser      parser_;
    NmeaPosition    dpos_;

    std::vector<char> device_buffer_;
    std::vector<char> simulate_buffer_;

    ros::Time last_nmea_sentence_;
    ros::Time last_correct_nmea_sentence_;
    double msg_count_;

    ros::Publisher  position_pub_,
                    satellites_pub_,
                    motion_pub_,
                    utc_pub_,
                    corrections_pub_,
                    height_pub_,
                    magn_declination_pub_,
                    status_pub_;

    gps::MsgGlobalPosition msg_position_;
    gps::MsgSatellites msg_sattelites_;
    gps::MsgUtc msg_utc_;
    gps::MsgCorrections msg_corrections_;
    gps::MsgHeight msg_height_;
    gps::MsgMagnDeclination msg_declination_;
    gps::MsgMotion msg_motion_;
    gps::MsgStatus msg_status_;
};

///@}