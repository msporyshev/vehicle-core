#include "compass/x-sens/compass.h"
#include <iostream>
#include <cmath>

#define PERIOD_UPDATE       0.01
#define PERIOD_PUBLISH      0.10

#define PI  3.14159265

using namespace std;

const string Compass::NODE_NAME = "compass";

Compass::Compass(CompassConfig config): config_(config), new_data_avalible_(false)
{
    com_handle_ = mti_.invalid_file_descriptor;

    compass_commands_.resize(static_cast<size_t>(CommandCode::CommandsSize));

    compass_commands_[static_cast<size_t>(CommandCode::Init)].command = "Init";
    compass_commands_[static_cast<size_t>(CommandCode::Init)].function = &MTI::MTI_reset;
    compass_commands_[static_cast<size_t>(CommandCode::CalibrationOn)].command = "Calibration on";
    compass_commands_[static_cast<size_t>(CommandCode::CalibrationOn)].function = &MTI::MTI_start_calibrate;
    compass_commands_[static_cast<size_t>(CommandCode::CalibrationOff)].command = "Calibration off";
    compass_commands_[static_cast<size_t>(CommandCode::CalibrationOff)].function = &MTI::MTI_stop_calibrate;
    compass_commands_[static_cast<size_t>(CommandCode::Reset)].command = "Reset";
    compass_commands_[static_cast<size_t>(CommandCode::Reset)].function = &MTI::MTI_reset;
    compass_commands_[static_cast<size_t>(CommandCode::StoreFilter)].command = "Store filter";
    compass_commands_[static_cast<size_t>(CommandCode::StoreFilter)].function = &MTI::MTI_store_filter;
}

void Compass::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    acceleration_pub_ = comm.advertise<compass::MsgCompassAcceleration>();
    angle_pub_        = comm.advertise<compass::MsgCompassAngle>();
    angle_raw_pub_        = comm.advertise<compass::MsgCompassAngleRaw>();
    angle_rate_pub_   = comm.advertise<compass::MsgCompassAngleRate>();
    magnetometer_pub_ = comm.advertise<compass::MsgCompassMagnetometer>();

    comm.subscribe_cmd<Compass, compass::CmdConfig>(&Compass::handle_message, this);
    comm.subscribe_cmd<Compass, compass::CmdDeclination>(&Compass::handle_message, this);
}

void Compass::start_timers(ipc::Communicator& comm)
{
    if(!config_.modelling) {
        timer_data_update_  = comm.create_timer(PERIOD_UPDATE, &Compass::data_update, this);
        timer_data_publish_ = comm.create_timer(PERIOD_PUBLISH, &Compass::data_publish, this);
    } else {
        timer_data_publish_model_ = comm.create_timer(PERIOD_PUBLISH, &Compass::data_publish_modelling, this);
    }

}

void Compass::data_update(const ros::TimerEvent& event)
{
    ROS_DEBUG_STREAM("Try to get new block");
    int status = mti_.MTI_get_one_package_block (com_handle_, &MTI_data_); // работает = 39 мс,
    // на этом и основывается выбор длительности цикла обмена сообщениями (TIMER_PERIOD)
    if(status < 0) {
        ROS_WARN_STREAM("Received bad data.");
        return;
    }

    new_data_avalible_ = true;
}

void Compass::data_publish(const ros::TimerEvent& event)
{
    if (!new_data_avalible_) {
        ROS_DEBUG_STREAM("No new data for publishing");
        return;
    }

    // Указатель на структуру с актуальными данными измерений, полученными с устройства
    MTI_struct *data;

    data = &MTI_data_;

    msg_acceleration_.header.stamp = ros::Time::now();
    msg_acceleration_.acc_x = data->accX;
    msg_acceleration_.acc_y = data->accY;
    msg_acceleration_.acc_z = data->accZ;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_acceleration_));
    acceleration_pub_.publish(msg_acceleration_);

    msg_angle_.header.stamp = ros::Time::now();
    msg_angle_.heading = data->yaw + config_.declination;    
    if (msg_angle_.heading < -180) {
        msg_angle_.heading += 360;
    } else if (msg_angle_.heading > 180) {
        msg_angle_.heading -= 360;
    }

    msg_angle_.pitch   = data->pitch;
    msg_angle_.roll    = data->roll;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_angle_));
    angle_pub_.publish(msg_angle_);

    msg_angle_raw_.header.stamp = ros::Time::now();
    
    // Компенсация влияния крена, дифферента на значение курса.
    double magn_compensate_x =  data->magX * cos(data->pitch * PI / 180) \
                             +  data->magY * sin(data->pitch * PI / 180) * sin(data->roll * PI / 180) \
                             +  data->magZ * cos(data->roll * PI / 180) * sin(data->pitch * PI / 180);
    
    double magn_compensate_y =  data->magY * cos(data->roll * PI / 180) \
                             -  data->magZ * sin(data->roll * PI / 180);

    msg_angle_raw_.heading = atan2(-magn_compensate_y, magn_compensate_x);

    if (msg_angle_raw_.heading < -180) {
        msg_angle_raw_.heading += 360;
    } else if (msg_angle_raw_.heading > 180) {
        msg_angle_raw_.heading -= 360;
    }

    msg_angle_raw_.pitch   = data->pitch;
    msg_angle_raw_.roll    = data->roll;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_angle_raw_));
    angle_raw_pub_.publish(msg_angle_raw_);

    msg_angle_rate_.header.stamp = ros::Time::now();
    msg_angle_rate_.rate_head  = data->gyrZ;
    msg_angle_rate_.rate_pitch = data->gyrY;
    msg_angle_rate_.rate_roll  = data->gyrX;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_angle_rate_));
    angle_rate_pub_.publish(msg_angle_rate_);

    msg_magnetometer_.header.stamp = ros::Time::now();
    msg_magnetometer_.magn_x = data->magX;
    msg_magnetometer_.magn_y = data->magY;
    msg_magnetometer_.magn_z = data->magZ;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_magnetometer_));
    magnetometer_pub_.publish(msg_magnetometer_);

    static bool is_first_run = true;
    if(is_first_run) {
        ROS_INFO_STREAM("True_Yaw\tMagn_Yaw\tPitch\tRoll\tVel_Yaw\tVel_Pitch\tVel_Roll");
        is_first_run = false;
    }

    ROS_INFO_STREAM(msg_angle_.heading << "\t" << msg_angle_raw_.heading << "\t" 
                 << msg_angle_.pitch << "\t" << msg_angle_.roll << "\t"
                 << msg_angle_rate_.rate_head << "\t" << msg_angle_rate_.rate_pitch << "\t"
                 << msg_angle_rate_.rate_roll);

    new_data_avalible_ = false;
}

void Compass::data_publish_modelling(const ros::TimerEvent& event)
{
    static float test_data = 0;
    
    if (test_data > 10) {
        test_data = 0;
    } else {
        test_data++;
    }

    // Заполнение структуры модельными данными
    msg_acceleration_.header.stamp = ros::Time::now();
    msg_acceleration_.acc_x = 100 * test_data + 0;
    msg_acceleration_.acc_y = 100 * test_data + 100;
    msg_acceleration_.acc_z = 100 * test_data + 200;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_acceleration_));
    acceleration_pub_.publish(msg_acceleration_);
        
    msg_angle_.header.stamp = ros::Time::now();
    msg_angle_.heading = 10 * test_data + 0;
    msg_angle_.pitch   = 10 * test_data + 10;
    msg_angle_.roll    = 10 * test_data + 20;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_angle_));
    angle_pub_.publish(msg_angle_);

    msg_angle_rate_.header.stamp = ros::Time::now();
    msg_angle_rate_.rate_head  = test_data + 0;
    msg_angle_rate_.rate_pitch = test_data + 5;
    msg_angle_rate_.rate_roll  = test_data + 10;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_angle_rate_));
    angle_rate_pub_.publish(msg_angle_rate_);
        
    msg_magnetometer_.header.stamp = ros::Time::now();
    msg_magnetometer_.magn_x = 100 * test_data + 0;
    msg_magnetometer_.magn_y = 100 * test_data + 100;
    msg_magnetometer_.magn_z = 100 * test_data + 200;
    ROS_DEBUG_STREAM("Published " << ipc::classname(msg_magnetometer_));
    magnetometer_pub_.publish(msg_magnetometer_);

    static bool is_first_run = true;
    if(is_first_run) {
        ROS_INFO_STREAM("True_Yaw\tMagn_Yaw\tPitch\tRoll\tVel_Yaw\tVel_Pitch\tVel_Roll");
        is_first_run = false;
    }

    ROS_INFO_STREAM(msg_angle_.heading << "\t" << msg_angle_raw_.heading << "\t" 
                 << msg_angle_.pitch << "\t" << msg_angle_.roll << "\t"
                 << msg_angle_rate_.rate_head << "\t" << msg_angle_rate_.rate_pitch << "\t"
                 << msg_angle_rate_.rate_roll);
}

void Compass::init_mti()
{
    if (config_.modelling) {
        ROS_INFO_STREAM("Modeling mode was enabled");
    } else {
        ROS_INFO_STREAM("Normal mode was enabled. (using real device)");
        ROS_INFO_STREAM("com name = " << config_.port);
        ROS_INFO_STREAM("baudrate = " << config_.baundrate);
        ROS_INFO_STREAM("declination = " << config_.declination);
    }

    // Если драйвер запущен в нормальном режиме (работа с реальным устройством)
    if(!config_.modelling) {
        com_handle_ = mti_.MTI_COM_open(config_.port.c_str(), config_.baundrate);

        if(com_handle_ == mti_.invalid_file_descriptor) {
            ROS_ERROR_STREAM("Error while connection");
            return;
        }

        ROS_DEBUG_STREAM("MTI_init was started");
        mti_.MTI_init(com_handle_);
        ROS_DEBUG_STREAM("MTI_init was finished");
    }
}

void Compass::close_mti()
{
    if(!config_.modelling) {
        mti_.MTI_COM_close(&com_handle_);
    }
}

void Compass::handle_message(const compass::CmdConfig& msg)
{
    ROS_DEBUG_STREAM("New config msg:");
    ROS_DEBUG_STREAM("command code: " << msg.command_code);

    if(msg.command_code >= static_cast<size_t>(CommandCode::CommandsSize)) {
        ROS_ERROR_STREAM("Command code range out from command vector");
        return;
    }

    ROS_INFO_STREAM("Receiving command: " << compass_commands_[msg.command_code].command);
    if(!config_.modelling) {
        void(MTI::*function)(int);
        function = compass_commands_[msg.command_code].function;
        (mti_.*function)(com_handle_);
    }
}

void Compass::handle_message(const compass::CmdDeclination& msg)
{
    ROS_DEBUG_STREAM("New declination msg:");
    ROS_DEBUG_STREAM("declination: " << msg.declination);
    config_.declination = msg.declination;
}