/**
\file
\brief Реализация драйвера GPS

В данном файле находятся реализации методов, объявленных в gps.h

\ingroup gps_node
*/

///@{

#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>

#include "gps/gps.h"
#include "gps/COM-ports.h"

#include "utils.h"

#define LF 10
#define CR 13

#define STATUS_PERIOD 1
#define HANDLE_PERIOD 0.1
#define SIMULATE_PERIOD 0.1
#define TIMEOUT (HANDLE_PERIOD / 2)
#define SLEEP_TIME (TIMEOUT / 10)

#define BUFFER_SIZE 100

#define KNOTS_TO_KPH 0.2778

using namespace std;

const string Gps::NODE_NAME = "gps";

Gps::Gps(GpsConfig config): config_(config)
{
    device_descriptor_ = -1;
    msg_count_ = 0;

    info_.smask = NMEALIB_SENTENCE_GPNON;
    nmeaInfoClear(&info_);
    nmeaParserInit(&parser_, 0);
}

void Gps::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений драйвера GPS
    */
    position_pub_          = comm.advertise<gps::MsgGlobalPosition>();
    satellites_pub_        = comm.advertise<gps::MsgSatellites>();
    utc_pub_               = comm.advertise<gps::MsgUtc>();
    corrections_pub_       = comm.advertise<gps::MsgCorrections>();
    height_pub_            = comm.advertise<gps::MsgHeight>();
    magn_declination_pub_  = comm.advertise<gps::MsgMagnDeclination>();
    status_pub_            = comm.advertise<gps::MsgStatus>();
    motion_pub_            = comm.advertise<gps::MsgMotion>();

    if(config_.simulating) {
        comm.create_timer(SIMULATE_PERIOD,  &Gps::simulate_gps_data, this);
    }

    comm.create_timer(HANDLE_PERIOD,  &Gps::handle_gps_data, this);
    comm.create_timer(STATUS_PERIOD,  &Gps::publish_status, this);
}

int Gps::open_device()
{
    char* port_name = const_cast<char*>(config_.port.c_str());
    int port_speed = config_.baudrate;

    if(!config_.simulating) {
        device_descriptor_ = COM_open(port_name);    
    } else {
        device_descriptor_ = 1;
    }

    if(device_descriptor_ == -1) {
        ROS_WARN_STREAM("Some problem with device opening. Check port name.");
        ROS_WARN_STREAM("Current port name: " << port_name);
        return -1;
    }

    if(!config_.simulating) {
        if(COM_settings(device_descriptor_, port_speed) != 0) {
            ROS_WARN_STREAM("Some problem with port settings.");
            ROS_WARN_STREAM("Current port name: " << port_name);
            return -1;
        }  
    } else {
        device_descriptor_ = 1;
    }


    ROS_DEBUG_STREAM("Device successfuly opened.");
    return 0;
}

int Gps::close_device()
{
    if(device_descriptor_ != -1) {
        ROS_INFO_STREAM("Device closed.");
        if(!config_.simulating) {
            return COM_close(device_descriptor_);
        } else {
            return 0;
        }
    } else {
        ROS_WARN_STREAM("Try closing, but device was not open.");
    }
}

void Gps::publish_gprms_data()
{
    if(info_.sig != NMEALIB_SIG_INVALID) {
        gps::MsgGlobalPosition msg_position;
        nmeaMathInfoToPosition(&info_, &dpos_);
        msg_position.longitude = dpos_.lon;
        msg_position.latitude  = dpos_.lat;
        msg_position.header.stamp = ros::Time::now();
        position_pub_.publish(msg_position);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_position));

        gps::MsgMotion msg_motion;
        msg_motion.velocity = info_.speed * KNOTS_TO_KPH;
        msg_motion.heading  = info_.track;
        msg_motion.velocity_east  = msg_motion.velocity * cos(utils::to_rad(msg_motion.heading));
        msg_motion.velocity_north = msg_motion.velocity * sin(utils::to_rad(msg_motion.heading));
        msg_motion.header.stamp = ros::Time::now();
        motion_pub_.publish(msg_motion);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_motion));

        gps::MsgUtc msg_utc;
        msg_utc.year = info_.utc.year;
        msg_utc.mon  = info_.utc.mon;
        msg_utc.day  = info_.utc.day;
        msg_utc.hour = info_.utc.hour;
        msg_utc.min  = info_.utc.min;
        msg_utc.sec  = info_.utc.sec;
        msg_utc.hsec = info_.utc.hsec;
        msg_utc.header.stamp = ros::Time::now();
        utc_pub_.publish(msg_utc);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_utc));

        gps::MsgMagnDeclination msg_declination;
        msg_declination.declination = info_.magvar;
        msg_declination.header.stamp = ros::Time::now();
        magn_declination_pub_.publish(msg_declination);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_declination));
    } else {
        ROS_WARN_STREAM("Status of message: " << nmeaInfoSignalToString(info_.sig) << ". We dont trust it.");
    }
}

void Gps::publish_gpgga_data()
{
    if(info_.sig != NMEALIB_SIG_INVALID) {
        if(info_.hdop > 0.0 && info_.hdop <= 100.0) {
            gps::MsgGlobalPosition msg_position;
            nmeaMathInfoToPosition(&info_, &dpos_);
            msg_position.longitude = dpos_.lon;
            msg_position.latitude  = dpos_.lat;
            msg_position.header.stamp = ros::Time::now();
            position_pub_.publish(msg_position);
            ROS_DEBUG_STREAM("Published " << ipc::classname(msg_position));
        }

        gps::MsgCorrections msg_corrections;
        msg_corrections.type = info_.sig;
        msg_corrections.dgps_time_elasped = info_.dgpsAge;
        msg_corrections.dgps_sid = info_.dgpsSid;
        msg_corrections.header.stamp = ros::Time::now();
        corrections_pub_.publish(msg_corrections);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_corrections));

        gps::MsgSatellites msg_satellites;
        msg_satellites.satellites = info_.satellites.inViewCount;
        msg_satellites.header.stamp = ros::Time::now();
        satellites_pub_.publish(msg_satellites);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_satellites));

        gps::MsgHeight msg_height_;
        msg_height_.elevation = info_.elevation;
        msg_height_.height = info_.height;
        msg_height_.header.stamp = ros::Time::now();
        height_pub_.publish(msg_height_);
        ROS_DEBUG_STREAM("Published " << ipc::classname(msg_height_));
    } else {
        ROS_WARN_STREAM("Status of message: " << nmeaInfoSignalToString(info_.sig) << ". We dont trust it.");
    }
}

void Gps::publish_status(const ros::TimerEvent& event)
{
    gps::MsgStatus msg_status;

    msg_status.last_msg_time = (ros::Time::now() - last_nmea_sentence_).toSec();
    msg_status.msg_counter   = msg_count_;
    msg_status.last_status   = info_.sig;
    msg_status.last_hdop     = info_.hdop;
    msg_status.header.stamp = ros::Time::now();
    status_pub_.publish(msg_status);
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_status));
}

bool Gps::nmea_decode(std::vector<char> buffer)
{
    info_.smask = NMEALIB_SENTENCE_GPNON;
    int parsed = nmeaParserParse(&parser_, buffer.data(), buffer.size(), &info_);
    ROS_DEBUG_STREAM("New state of info_.smask: " << info_.smask);

    if(parsed > 0) {
        msg_count_ += parsed;
        last_nmea_sentence_ = ros::Time::now();
        if((info_.smask & NMEALIB_SENTENCE_GPRMC) == NMEALIB_SENTENCE_GPRMC) {
            ROS_DEBUG_STREAM("GPRMS sentence was received ");
            publish_gprms_data();
        }
        if((info_.smask & NMEALIB_SENTENCE_GPGGA) == NMEALIB_SENTENCE_GPGGA) {
            ROS_DEBUG_STREAM("GPGGA sentence was received ");
            publish_gpgga_data();
        }
        if(info_.smask == NMEALIB_SENTENCE_GPNON) {
            ROS_WARN_STREAM("For some reason buffer was parsed, but no strings inside");
        }
        return true;
    }

    ROS_WARN_STREAM("For some reason buffer was not parsed");
    return false;
}

bool Gps::get_data(int dd, std::vector<char>& buffer)
{
    if(!config_.simulating) {
        char c_buffer[BUFFER_SIZE];
        int read_count = read(dd, c_buffer, BUFFER_SIZE);
        if(read_count > 0) {
            vector<char> temp(c_buffer, c_buffer + read_count);
            copy(temp.begin(), temp.end(), std::back_inserter(buffer));
            return true;
        } else if(read_count < 0) {
            ROS_ERROR_STREAM("Some problem with port. Cannot read from it.");
            return false;
        } else {
            ROS_WARN_STREAM("Nothing to read.");
            return false;
        }
    } else {
        copy(simulate_buffer_.begin(), simulate_buffer_.end(), std::back_inserter(buffer));
        simulate_buffer_.clear();
        return true;
    }
}

void Gps::handle_gps_data(const ros::TimerEvent& event)
{
    ROS_ASSERT_MSG(device_descriptor_ != -1, "Port was not configure. Emergency exit.");
    
    int dev_buffer_size = device_buffer_.size();

    ros::Time start_time = ros::Time::now();
    while(ros::Time::now() < start_time + ros::Duration(TIMEOUT)){
        get_data(device_descriptor_, device_buffer_);
        ros::Duration(SLEEP_TIME).sleep();
    }

    dev_buffer_size = device_buffer_.size() - dev_buffer_size;
    ROS_DEBUG("From buffer was readed %d bytes", dev_buffer_size);
    
    if(device_buffer_.size() > 2) {
        reverse(device_buffer_.begin(),device_buffer_.end());
        
        auto it = device_buffer_.begin();
        for(it = device_buffer_.begin(); it != device_buffer_.end(); it++) {
            if(it < (device_buffer_.end() - 1)) {
                if(*(it) == LF && *(it + 1) == CR) {
                    break;
                }
            }
        }
        
        vector<char> handle_ready_buffer(it, device_buffer_.end());
        reverse(handle_ready_buffer.begin(),handle_ready_buffer.end());
        nmea_decode(handle_ready_buffer);

        device_buffer_.erase(it, device_buffer_.end());
        if(device_buffer_.size() > 0) {
            reverse(device_buffer_.begin(),device_buffer_.end());            
        }
    }
}

void Gps::simulate_gps_data(const ros::TimerEvent& event) {

    NmeaInfo info;
    NmeaTime utc;
    struct timeval time;
    
    static double lat = 43.0, lon = 131.0, elevation = 0, hdop = 0, speed = 5, track = 0;
    static int dgpsAge = 0, dgpsSid = 0;

    lat = lat < 45 ? lat + 0.001 : 43;
    lon = lon < 133 ? lon + 0.001 : 131;
    elevation = elevation < 50 ? elevation + 0 : 0;
    hdop = hdop < 200 ? hdop + 5 : 0;
    speed = speed < 10 ? speed + 0.1 : 5;
    track = track < 90 ? track + 0.1 : 0;
    dgpsAge = dgpsAge < 30 ? dgpsAge + 1 : 0;
    dgpsSid = dgpsSid < 100 ? dgpsSid + 1 : 0;


    gettimeofday(&time, NULL);
    nmeaTimeSet(&utc, &(info.present), &time);
    
    nmeaInfoClear(&info);
    
    NmeaSatellites sat;
    sat.inUseCount = 1;
    sat.inUse[0] = 10;
    sat.inViewCount = 1;
    sat.inView[0].prn = 111;
    sat.inView[0].elevation = 88;
    sat.inView[0].azimuth = 125;
    sat.inView[0].snr = 15;

    NmeaProgress progress;
    progress.gpgsvInProgress = true;

    info.present = NMEALIB_INFO_PRESENT_MASK;
    info.utc = utc;
    info.sig = NMEALIB_SIG_FIX;
    info.pdop = 5.0;
    info.hdop = hdop;
    info.vdop = 5.0;
    info.latitude = nmeaMathDegreeToNdeg(lat);
    info.longitude = nmeaMathDegreeToNdeg(lon);
    info.elevation = elevation;
    info.height = 3.4;
    info.speed = speed;
    info.track = track;
    info.mtrack = 100;
    info.magvar = -10;
    info.dgpsAge = dgpsAge;
    info.dgpsSid = dgpsSid;
    info.satellites = sat;
    info.progress = progress;
    info.metric = true;

    NmeaGPGGA gga_pack;
    char gga_sentence[150];

    nmeaGPGGAFromInfo(&info, &gga_pack);
    int gga_size = nmeaGPGGAGenerate(gga_sentence, 150, &gga_pack);
    // gga_sentence[gga_size] = CR;
    // gga_sentence[gga_size + 1] = LF;
    // gga_size += 2;
    vector<char> gga_temp(gga_sentence, gga_sentence + gga_size);
    ROS_DEBUG_STREAM("GGA temp size: " << gga_temp.size());

    NmeaGPRMC rmc_pack;
    char rmc_sentence[150];

    nmeaGPRMCFromInfo(&info, &rmc_pack);
    int rmc_size = nmeaGPRMCGenerate(rmc_sentence, 150, &rmc_pack);
    // rmc_sentence[rmc_size] = CR;
    // rmc_sentence[rmc_size + 1] = LF;
    // rmc_size += 2;
    vector<char> rmc_temp(rmc_sentence, rmc_sentence + rmc_size);
    ROS_DEBUG_STREAM("RMS temp size: " << rmc_temp.size());

    copy(gga_temp.begin(), gga_temp.end(), std::back_inserter(simulate_buffer_));
    copy(rmc_temp.begin(), rmc_temp.end(), std::back_inserter(simulate_buffer_));
    ROS_DEBUG_STREAM("Simulate buffer size: " << simulate_buffer_.size());

    std::stringstream ss;
    for(int i = 0; i < simulate_buffer_.size(); i++) {
        ss << simulate_buffer_[i];
    }
    ROS_DEBUG_STREAM("Simulate string: " << ss.str());
}
///@}