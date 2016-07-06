/**
\file
\brief Заголовочный файл драйвера супервизора

В данном файле находится обьявление класса драйвера супервизора

\ingroup supervisor_node
*/

///@{

#pragma once

#include <string>
// #include <yaml_reader.h>/

#include <libipc/ipc.h>
#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>

#include <supervisor/SupervisorDevicesConfig.h>
#include <supervisor/SupervisorEthernetConfig.h>
#include <supervisor/SupervisorCalibrationConfig.h>
#include <supervisor/SupervisorPeriodsConfig.h>

#include "supervisor/supervisor_headers.h"
#include "supervisor/supervisor_devices.h"

#include "supervisor/MsgLeak.h"
#include "supervisor/MsgCompensator.h"
#include "supervisor/MsgDevicesStatus.h"
#include "supervisor/MsgShortCircuit.h"
#include "supervisor/MsgAdc.h"
#include "supervisor/MsgExternalAdc.h"
#include "supervisor/MsgDepth.h"
#include "supervisor/MsgPowerSupplyState.h"
#include "supervisor/MsgTemperature.h"
#include "supervisor/MsgWaterState.h"

#include "supervisor/CmdCan.h"
#include "supervisor/CmdConfigureUdp.h"
#include "supervisor/CmdDeviceKey.h"
#include "supervisor/CmdFirmware.h"
#include "supervisor/CmdPwm.h"
#include "supervisor/CmdSystemFlags.h"
#include "supervisor/CmdUart.h"

#include "eth_connection.h"

#define MIN_TIME_DISCRETE   0.200

struct LinearCalibrator
{
    float x1, y1, x2, y2;
    
    float calibrate(float x)
    {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
};

struct Device {
    int position;
    int state;
};

struct timedDepth {
    float depth;
    double time;
};

class Supervisor
{

public:
    Supervisor(bool is_simulating = false);
    ~Supervisor();
    const static std::string NODE_NAME;

    void publish_leak(const ros::TimerEvent& event);
    void publish_compensator(const ros::TimerEvent& event);
    void publish_devices_status(const ros::TimerEvent& event);
    void publish_short_circuit(const ros::TimerEvent& event);
    void publish_adc(const ros::TimerEvent& event);
    void publish_external_adc(const ros::TimerEvent& event);
    void publish_depth(const ros::TimerEvent& event);
    void publish_temperature(const ros::TimerEvent& event);
    void publish_power_supply_state(const ros::TimerEvent& event);

    void water_check(const ros::TimerEvent& event);
    void connection_heartbeat(const ros::TimerEvent& event);

    void init_connection(ipc::Communicator& communicator);
    void start_timers(ipc::Communicator& communicator);

    void set_config_ethernet(supervisor::SupervisorEthernetConfig& config, unsigned int level);
    void set_config_calibration(supervisor::SupervisorCalibrationConfig& config, unsigned int level);
    void set_config_periods(supervisor::SupervisorPeriodsConfig& config, unsigned int level);
    void set_config_devices(supervisor::SupervisorDevicesConfig& config, unsigned int level);

private:

    void handle_message(const supervisor::CmdCan& msg);
    void handle_message(const supervisor::CmdConfigureUdp& msg);
    void handle_message(const supervisor::CmdDeviceKey& msg);
    void handle_message(const supervisor::CmdFirmware& msg);
    void handle_message(const supervisor::CmdPwm& msg);
    void handle_message(const supervisor::CmdSystemFlags& msg);
    void handle_message(const supervisor::CmdUart& msg);

    void udp_data_parser(const ros::TimerEvent& event);

    void marshall_and_publish(std::vector<unsigned char> data, TcpHeaders ID);
    void recognize_udp_header(UdpHeaders Id, std::vector<unsigned char> data);

    void calculate_depth_velocity();

    unsigned short from_bitmask(std::vector<unsigned char> data);
    void bitmask(std::vector<unsigned char>& data, std::vector<unsigned char> input_data_array);

    void handler_udp_leak           (std::vector<unsigned char> data);
    void handler_udp_firmware_ready (std::vector<unsigned char> data);
    void handler_udp_devices_status (std::vector<unsigned char> data);
    void handler_udp_short_circuit  (std::vector<unsigned char> data);
    void handler_udp_adc            (std::vector<unsigned char> data);
    void handler_udp_external_adc   (std::vector<unsigned char> data);
    void handler_udp_compensator    (std::vector<unsigned char> data);

    void get_simulating_data(std::vector<unsigned char>& data);

    bool is_simulating_;

    timedDepth prev_depth_;
    float depth_velocity_;
    float depth_delta_time_;
    float depth_velocity_temp_;

    LinearCalibrator lc_depth_;
    LinearCalibrator lc_current_;
    LinearCalibrator lc_battery_volts_;
    LinearCalibrator lc_temperature_;
    LinearCalibrator lc_csw_volts_;

    ros::Publisher  leak_pub_,
                    compensator_pub_,
                    devices_status_pub_,
                    short_circuit_pub_,
                    adc_pub_,
                    external_adc_pub_,
                    depth_pub_,
                    power_supply_pub_,
                    temperature_pub_,
                    water_check_pub_;

    ros::Timer      timer_leak_,
                    timer_compensator_,
                    timer_devices_status_,
                    timer_short_circuit_,
                    timer_adc_,
                    timer_external_adc_,
                    timer_depth_,
                    timer_udp_parser_,
                    timer_heartbeat_,
                    timer_power_supply_,
                    timer_temperature_,
                    timer_water_check_;

    supervisor::MsgLeak msg_leak_;
    supervisor::MsgCompensator msg_comp_;
    supervisor::MsgDevicesStatus msg_devices_status_;
    supervisor::MsgShortCircuit msg_short_circuit_;
    supervisor::MsgAdc msg_adc_;
    supervisor::MsgExternalAdc msg_adc_ext_;
    supervisor::MsgDepth msg_depth_;
    supervisor::MsgPowerSupplyState msg_power_supply_;
    supervisor::MsgTemperature msg_temperature_;
    supervisor::MsgWaterState msg_water_check_;

    EthConnection* eth_connection_;

    supervisor::SupervisorEthernetConfig config_ethernet_;
    supervisor::SupervisorCalibrationConfig config_calibration_;
    supervisor::SupervisorPeriodsConfig config_periods_;
    supervisor::SupervisorDevicesConfig config_devices_;

    std::vector<Device> devices;

    ros::NodeHandle* node_settings_ethernet_;
    ros::NodeHandle* node_settings_calibration_;
    ros::NodeHandle* node_settings_periods_;
    ros::NodeHandle* node_settings_devices_;

    dynamic_reconfigure::Server<supervisor::SupervisorEthernetConfig>* server_settings_ethernet_;
    dynamic_reconfigure::Server<supervisor::SupervisorCalibrationConfig>* server_settings_calibration_;
    dynamic_reconfigure::Server<supervisor::SupervisorPeriodsConfig>* server_settings_periods_;
    dynamic_reconfigure::Server<supervisor::SupervisorDevicesConfig>* server_settings_devices_;

    dynamic_reconfigure::Server<supervisor::SupervisorEthernetConfig>::CallbackType callback_ethernet_;
    dynamic_reconfigure::Server<supervisor::SupervisorCalibrationConfig>::CallbackType callback_calibration_;
    dynamic_reconfigure::Server<supervisor::SupervisorPeriodsConfig>::CallbackType callback_periods_;
    dynamic_reconfigure::Server<supervisor::SupervisorDevicesConfig>::CallbackType callback_devices_;
};

///@}