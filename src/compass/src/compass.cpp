/**
\file
\brief Main-файл работы с компасом

В данном файле находится основной цикл работы компасса,
происходит инициализация системы, подключение к ROS-core,
сьем данных и рассылка с помощью ROS-пакета

\defgroup compass_node Нод компаса
\brief Данный нод предназначен для сьема данных с компаса и публикации в сеть

*/

#include <stdio.h>              // Standard Input / Output
#include <stdarg.h>             // Variable Arguments
#include <string>               // Работа с переменными типа string
#include <vector>               // Библиотека для работы с vector
#include <signal.h>
#include <iomanip>
#include <cmath>

#include <libipc/ipc.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <boost/program_options.hpp>
#include <yaml_reader.h>                    // библиотека для парсинга yaml конфигов
#include "compass/driver_compass.h"         // набор функций для работы с компасом
#include "compass/settings.h"

#include "compass/MsgCompassAcceleration.h"
#include "compass/MsgCompassAngle.h"
#include "compass/MsgCompassAngleRate.h"
#include "compass/MsgCompassMagnetometer.h"
#include "compass/MsgCompassQuaternion.h"
//------------------------------------------------------------------------------
//Defines

#define LOOP_RATE 100

#define HANDLE_DATA_PERIOD      0.010
#define BROADCAST_DATA_PERIOD   0.100

#define HISTORY_SIZE    10

using namespace std;
namespace po = boost::program_options;

const string node_name = "compass";
#define CONFIG_FILENAME "compass.yml"

//------------------------------------------------------------------------------
//Functions

void read_config_data();  // Парсинг данных из конфиг файла
void safety_exit();

string get_autoname();
//------------------------------------------------------------------------------

string com_name;               // Имя COM-порта, к которому подключено устройство
int com_baudrate;              // Скорость обмена по COM-порту

bool work_state = false;       // Состояние работы Копаса
bool raw_mode = false;
bool debug = false;
bool calibration_mode = false;
bool modelling_mode = false;
bool configuration_mode = false;

Compass_settings settings;

void program_options_init(int argc, char** argv) 
{
    po::options_description desc("Usage");
    desc.add_options()
      ("help,h", "Produce help message.")
      ("port,c", po::value(&com_name),
          "Set COM-port name (e.g. /dev/ttyUSB0).")
      ("baundrate,b", po::value(&com_baudrate),
          "Set COM-port baundrate.")
      ("start_now,s", po::value(&work_state)->zero_tokens(),
          "Start echosounder right now.")
      ("debug,v", po::value(&debug)->zero_tokens(),
          "Show some usefull info.")
      ("modelling,o", po::value(&modelling_mode)->zero_tokens(),
          "Set modelling mode.")
      ("magnetic_calibration,l", po::value(&calibration_mode)->zero_tokens(),
          "Start calibration of magnetic sensor.")
      ("configuration,f", po::value(&configuration_mode)->zero_tokens(),
          "Set new configuration of sensor from yaml-file.");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(EXIT_SUCCESS);
    }

    if(debug)
        set_debug_level(debug);
}

std::vector<Rotation_t> rotation_history;
std::vector<Component_t> accelerometer_history;
std::vector<Component_t> gyroscope_history;
std::vector<Component_t> magnetometer_history;
std::vector<Quaternion_t> quaternion_history;

void handle_new_data(const ros::TimerEvent& event)
{
    if(!work_state)
        return;

    if(modelling_mode)
        return;

    Rotation_t   rotation;
    Component_t  accelerometer;
    Component_t  gyroscope;
    Component_t  magnetometer;
    Quaternion_t quaternion;

    int status;

    status = Get_Rotation_Data(&rotation);
    if(status == 0) {
        if(rotation_history.size() > HISTORY_SIZE) {
            rotation_history.erase(rotation_history.begin());
        }
        rotation_history.push_back(rotation);
    }

    status = Get_Accelerometer_Data(&accelerometer);
    if(status == 0) {
        if(accelerometer_history.size() > HISTORY_SIZE) {
            accelerometer_history.erase(accelerometer_history.begin());
        }
        accelerometer_history.push_back(accelerometer);
    }

    status = Get_Gyroscope_Data(&gyroscope);
    if(status == 0) {
        if(gyroscope_history.size() > HISTORY_SIZE) {
            gyroscope_history.erase(gyroscope_history.begin());
        }
        gyroscope_history.push_back(gyroscope);
    }

    status = Get_Magnetometer_Data(&magnetometer);
    if(status == 0) {
        if(magnetometer_history.size() > HISTORY_SIZE) {
            magnetometer_history.erase(magnetometer_history.begin());
        }
        magnetometer_history.push_back(magnetometer);
    }

    status = Get_Quaternion_Data(&quaternion);
    if(status == 0) {
        if(quaternion_history.size() > HISTORY_SIZE) {
            quaternion_history.erase(quaternion_history.begin());
        }
        quaternion_history.push_back(quaternion);
    }

}

float get_average(std::vector<float> data)
{
    sort(data.begin(), data.end());
    return data[static_cast<size_t>(data.size() / 2)];
}

Rotation_t get_rotation_average(vector<Rotation_t> vector_data)
{
    Rotation_t average;
    vector<float> heading, pitch, roll;
    
    for(auto &elem: vector_data) {
        heading.push_back(elem.Yaw);
        pitch.push_back(elem.Pitch);
        roll.push_back(elem.Roll);
    }

    average.Yaw = get_average(heading);
    average.Pitch = get_average(pitch);
    average.Roll = get_average(roll);

    return average;
}

ros::Publisher acceleration_pub;
ros::Publisher angle_pub;
ros::Publisher angle_rate_pub;
ros::Publisher magnetometer_pub;
ros::Publisher quaterniom_pub;

void publish_data(const ros::TimerEvent& event)
{
    if(!work_state)
        return;

    compass::MsgCompassAcceleration  msg_acceleration;
    compass::MsgCompassAngle         msg_angle;
    compass::MsgCompassAngleRate     msg_angle_rate;
    compass::MsgCompassMagnetometer  msg_magnetometer;
    compass::MsgCompassQuaternion    msg_quaternion;

    Rotation_t rotation;
    Component_t accelerometer;
    Component_t gyroscope;
    Component_t magnetometer;
    Quaternion_t quaternion;

    if(!modelling_mode) {

        if(rotation_history.size() > 0) {
            rotation = get_rotation_average(rotation_history);
            rotation_history.erase(rotation_history.begin());

            msg_angle.heading = rotation.Yaw;
            msg_angle.pitch   = rotation.Pitch;
            msg_angle.roll    = rotation.Roll;
            angle_pub.publish(msg_angle);
        }

        if(accelerometer_history.size() > 0) {
            accelerometer   = accelerometer_history.back();
            accelerometer_history.erase(accelerometer_history.begin());
            
            msg_acceleration.acc_x = accelerometer.X;
            msg_acceleration.acc_y = accelerometer.Y;
            msg_acceleration.acc_z = accelerometer.Z;
            acceleration_pub.publish(msg_acceleration);
        }

        if(gyroscope_history.size() > 0) {
            gyroscope = gyroscope_history.back();
            gyroscope_history.erase(gyroscope_history.begin());

            msg_angle_rate.rate_head  = gyroscope.X;
            msg_angle_rate.rate_pitch = gyroscope.Y;
            msg_angle_rate.rate_roll  = gyroscope.Z;
            angle_rate_pub.publish(msg_angle_rate);
        } 

        if(magnetometer_history.size() > 0) {
            magnetometer        = magnetometer_history.back();
            magnetometer_history.erase(magnetometer_history.begin());

            msg_magnetometer.magn_x = magnetometer.X;
            msg_magnetometer.magn_y = magnetometer.Y;
            msg_magnetometer.magn_z = magnetometer.Z;
            magnetometer_pub.publish(msg_magnetometer);
        }

        if(quaternion_history.size() > 0) {
            quaternion      = quaternion_history.back();
            quaternion_history.erase(quaternion_history.begin());

            msg_quaternion.Q1 = quaternion.Q1;
            msg_quaternion.Q2 = quaternion.Q2;
            msg_quaternion.Q3 = quaternion.Q3;
            msg_quaternion.Q4 = quaternion.Q4;
            quaterniom_pub.publish(msg_quaternion);
        }

    } else {
        // Заполнение структуры модельными данными
        msg_acceleration.acc_x = 300;
        msg_acceleration.acc_y = 400;
        msg_acceleration.acc_z = 500;
        acceleration_pub.publish(msg_acceleration);        
        
        msg_angle.heading = 30;
        msg_angle.pitch   = 45;
        msg_angle.roll    = 60;
        angle_pub.publish(msg_angle);

        msg_angle_rate.rate_head = 2;
        msg_angle_rate.rate_pitch = 3;
        msg_angle_rate.rate_roll = 4;
        angle_rate_pub.publish(msg_angle_rate);        
        
        msg_magnetometer.magn_x = 1000;
        msg_magnetometer.magn_y = 200;
        msg_magnetometer.magn_z = 100;
        magnetometer_pub.publish(msg_magnetometer);        
        
        msg_quaternion.Q1 = 0.1;
        msg_quaternion.Q2 = 0.2;
        msg_quaternion.Q3 = 0.3;
        msg_quaternion.Q4 = 0.4;
        quaterniom_pub.publish(msg_quaternion);
    }


    static bool print_header = 0;
    static double start_time;
    if(!print_header){
        start_time = ros::Time::now().toSec();
        cout << "Time\tTime_msec\tHead_degree\tPitch_degree\tRoll_degree\tHead_Rate_degree/s\t\
        Pitch_Rate_degree/s\tRoll_Rate_degree/s\t  acceleration_x_mg/s\t  acceleration_y_mg/s\t  acceleration_z_mg/s" << endl;
        print_header = 1;
    }
    LOG << \
    (ros::Time::now().toSec() - start_time) * 1000 << "\t" << \
    msg_angle.heading << "\t" << \
    msg_angle.pitch   << "\t" << \
    msg_angle.roll    << "\t" << \
    msg_angle_rate.rate_head  << "\t" << \
    msg_angle_rate.rate_pitch << "\t" << \
    msg_angle_rate.rate_roll  << "\t" << \
    msg_acceleration.acc_x << "\t" << \
    msg_acceleration.acc_y << "\t" << \
    msg_acceleration.acc_z << endl;
}

void read_config_data()
{
    YamlReader cfg;
    cout << "Load config file: " << CONFIG_FILENAME << endl;
    cfg.add_source(CONFIG_FILENAME);

    YAML::Node connection_cfg_node;
    cfg.read_param(connection_cfg_node, "connection_config");
    YamlReader connection_cfg(connection_cfg_node);

    connection_cfg.read_param(com_name, "com_port");
    connection_cfg.read_param(com_baudrate, "com_baudrate");

    settings.load_config(cfg);

    LOG << "default COM port: " << com_name << endl;
    LOG << "default COM port baudrate: " << com_baudrate << endl;
}

void safety_exit()
{
    if(!modelling_mode && (get_comport_status() == 0)) {
        bool status;
        status = INEMO_M1_Stop_Acquisition();
        if(debug)
            cout << "Status of INEMO_M1_Stop_Acquisition: " << status << endl;
        status = INEMO_M1_Disconnect();
        if(debug)
            cout << "Status of INEMO_M1_Disconnect: " << status << endl;
        status = close_compass();
        if(debug)
            cout << "Status of close_compass: " << status << endl;
    }
    cout << "Exiting..." << endl;
}

int main ( int argc, char *argv[] )
{
    // Вывод заголовка таблиц данных
    // read_config_data();
    //Обработка параметров запуска модуля
    program_options_init(argc, argv);

    //Инициализация ROS
    auto comm = ipc::init(argc, argv, node_name);

    acceleration_pub = comm.advertise<compass::MsgCompassAcceleration>();
    angle_pub        = comm.advertise<compass::MsgCompassAngle>();
    angle_rate_pub   = comm.advertise<compass::MsgCompassAngleRate>();
    magnetometer_pub = comm.advertise<compass::MsgCompassMagnetometer>();
    quaterniom_pub   = comm.advertise<compass::MsgCompassQuaternion>();

    LOG << "current COM-port: " << com_name << endl;
    LOG << "current Baudrate: " << com_baudrate << endl;

    if(!modelling_mode) {
        wait(POWER_ON_WAIT);
        bool com_status = open_compass(com_name.c_str(), com_baudrate);
        //Если не удалось открыть и настроить COM-порт
        if (com_status != 0) {
            return (EXIT_FAILURE);
        }
    }

    if(work_state && !modelling_mode) {
        compass_config_struct st;
        bool status;
        set_compass_config(st);
        status = INEMO_M1_Connect();
        if(status != 0) {
            LOG << "INEMO_M1_Connect return 1." << endl;
            safety_exit();
        }
        status = INEMO_M1_Config_Output();
        if(status != 0) {
            LOG << "INEMO_M1_Config_Output return 1." << endl;
            safety_exit();
        }
        
        status = settings.load_settings_flash();
        if(status != 0) {
            LOG << "Loading configuration return 1." << endl;
            safety_exit();
        }

        status = INEMO_M1_Start_Acquisition();
        if(status != 0) {
            LOG << "INEMO_M1_Start_Acquisition return 1." << endl;
            safety_exit();
        }

    } else if(calibration_mode && !modelling_mode) {
        INEMO_M1_Connect();
        int calibration_status;
        float calibration_timeout_s = 1000;
        settings.get_all_settings();
        calibration_status = start_calibration(calibration_timeout_s);
        if(calibration_status == 0) {
            LOG << "Calibration successfull." << endl;
        } else {
            LOG << "Calibration unsuccessfull." << endl;
        }
        settings.get_all_settings();
        settings.save_settings_flash();
        safety_exit();

    } else if (configuration_mode && !modelling_mode) {
        INEMO_M1_Connect();
        if(debug)
            settings.get_all_settings();
        settings.set_all_settings();
        settings.save_settings_flash();
        if(debug)
            settings.get_all_settings();
        safety_exit();

    }
    else if(modelling_mode) {
        cout << "Modeling mode was enabled" << endl;
    }
    else {
        cout << "Waiting IPC command for start." << endl;
    }

    ros::NodeHandle node = comm.get_node();

    ros::Timer handle_new_data_timer = node.createTimer(ros::Duration(HANDLE_DATA_PERIOD), handle_new_data);
    ros::Timer publish_data_timer    = node.createTimer(ros::Duration(BROADCAST_DATA_PERIOD), publish_data);

    ros::spin();

    safety_exit();
    return ( EXIT_SUCCESS );
}
