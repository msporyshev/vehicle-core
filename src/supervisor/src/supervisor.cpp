/**
\file
\brief Реализация драйвера супервизора

В данном файле находятся реализации методов, объявленных в supervisor.h

\ingroup supervisor_node
*/

///@{

#include <iostream>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <iomanip>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include "supervisor/supervisor.h"

using namespace std;

const string Supervisor::NODE_NAME = "supervisor";

Supervisor::Supervisor(bool is_simulating)
{
    is_simulating_ = is_simulating;

    eth_connection = new EthConnection();

    devices.resize(static_cast<size_t>(SupervisorDevices::Devices_size));

    node_settings_ethernet = new ros::NodeHandle("~/Ethernet");
    node_settings_calibration = new ros::NodeHandle("~/Calibration");
    node_settings_periods = new ros::NodeHandle("~/Periods");
    node_settings_devices = new ros::NodeHandle("~/Devices");

    server_settings_ethernet = new dynamic_reconfigure::Server<supervisor::SupervisorEthernetConfig>(*node_settings_ethernet);
    server_settings_calibration = new dynamic_reconfigure::Server<supervisor::SupervisorCalibrationConfig>(*node_settings_calibration);
    server_settings_periods = new dynamic_reconfigure::Server<supervisor::SupervisorPeriodsConfig>(*node_settings_periods);
    server_settings_devices = new dynamic_reconfigure::Server<supervisor::SupervisorDevicesConfig>(*node_settings_devices);
    
    callback_ethernet = boost::bind(&Supervisor::set_config_ethernet, this, _1, _2);
    callback_calibration = boost::bind(&Supervisor::set_config_calibration, this, _1, _2);
    callback_periods = boost::bind(&Supervisor::set_config_periods, this, _1, _2);
    callback_devices = boost::bind(&Supervisor::set_config_devices, this, _1, _2);

    server_settings_ethernet->setCallback(callback_ethernet);
    server_settings_calibration->setCallback(callback_calibration);
    server_settings_periods->setCallback(callback_periods);
    server_settings_devices->setCallback(callback_devices);
}

Supervisor::~Supervisor()
{
    delete eth_connection;
    delete node_settings_ethernet;
    delete node_settings_calibration;
    delete node_settings_periods;
    delete node_settings_devices;
    delete server_settings_ethernet;
    delete server_settings_calibration;
    delete server_settings_periods;
    delete server_settings_devices;
}

void Supervisor::init_connection(ipc::Communicator& comm)
{
    /**
        Регистрация всех исходящих сообщений навига
    */
    leak_pub_           = comm.advertise<supervisor::MsgLeak>();
    compensator_pub_    = comm.advertise<supervisor::MsgCompensator>();
    devices_status_pub_ = comm.advertise<supervisor::MsgDevicesStatus>();
    short_circuit_pub_  = comm.advertise<supervisor::MsgShortCircuit>();
    adc_pub_            = comm.advertise<supervisor::MsgAdc>();
    external_adc_pub_   = comm.advertise<supervisor::MsgExternalAdc>();
    depth_pub_          = comm.advertise<supervisor::MsgDepth>();
    power_supply_pub_   = comm.advertise<supervisor::MsgPowerSupplyState>();
    temperature_pub_    = comm.advertise<supervisor::MsgTemperature>();
    water_check_pub_    = comm.advertise<supervisor::MsgWaterState>();

    /**
        Подписка на сообщения
    */

    comm.subscribe_cmd<Supervisor, supervisor::CmdCan>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdConfigureUdp>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdDeviceKey>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdFirmware>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdPwm>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdSystemFlags>(&Supervisor::handle_message, this);
    comm.subscribe_cmd<Supervisor, supervisor::CmdUart>(&Supervisor::handle_message, this);
}

void Supervisor::start_timers(ipc::Communicator& communicator)
{
    timer_leak             = communicator.create_timer(config_periods.period_leak,           &Supervisor::publish_leak, this);
    timer_compensator      = communicator.create_timer(config_periods.period_compensator,    &Supervisor::publish_compensator, this);
    timer_devices_status   = communicator.create_timer(config_periods.period_devices_status, &Supervisor::publish_devices_status, this);
    timer_short_circuit    = communicator.create_timer(config_periods.period_short_circuit,  &Supervisor::publish_short_circuit, this);
    timer_adc              = communicator.create_timer(config_periods.period_adc,            &Supervisor::publish_adc, this);
    timer_external_adc     = communicator.create_timer(config_periods.period_external_adc,   &Supervisor::publish_external_adc, this);
    timer_depth            = communicator.create_timer(config_periods.period_depth,          &Supervisor::publish_depth, this);
    timer_power_supply     = communicator.create_timer(config_periods.period_power_supply,   &Supervisor::publish_power_supply_state, this);
    timer_temperature      = communicator.create_timer(config_periods.period_temperature,    &Supervisor::publish_temperature, this);
    timer_water_check      = communicator.create_timer(config_periods.period_water_check,    &Supervisor::water_check, this);
    
    timer_heartbeat        = communicator.create_timer(config_periods.period_heartbeat,      &Supervisor::connection_heartbeat, this);
    timer_udp_parser       = communicator.create_timer(config_periods.period_parser,         &Supervisor::udp_data_parser, this);
}

void Supervisor::set_config_ethernet(supervisor::SupervisorEthernetConfig& config, unsigned int level)
{
    ROS_INFO_STREAM("Received new config ethernet data");

    EthSettings eth_settings;
    eth_settings.ip_address         = config.eth_address;
    eth_settings.tcp_port           = config.tcp_port;
    eth_settings.udp_port           = config.udp_port;
    eth_settings.period_tcp_send    = config.tcp_period;
    eth_settings.period_udp_listen  = config.udp_period;
    eth_connection->init_connection_settings(eth_settings);
    config_ethernet = config;
}

void Supervisor::set_config_calibration(supervisor::SupervisorCalibrationConfig& config, unsigned int level)
{
    ROS_INFO_STREAM("Received new config calibration data");

    lc_depth.x1 = config.depth_x1;
    lc_depth.y1 = config.depth_y1;
    lc_depth.x2 = config.depth_x2;
    lc_depth.y2 = config.depth_y2;

    lc_current.x1 = config.current_x1;
    lc_current.y1 = config.current_y1;
    lc_current.x2 = config.current_x2;
    lc_current.y2 = config.current_y2;

    lc_battery_volts.x1 = config.battery_x1;
    lc_battery_volts.y1 = config.battery_y1;
    lc_battery_volts.x2 = config.battery_x2;
    lc_battery_volts.y2 = config.battery_y2;

    lc_temperature.x1 = config.temperature_x1;
    lc_temperature.y1 = config.temperature_y1;
    lc_temperature.x2 = config.temperature_x2;
    lc_temperature.y2 = config.temperature_y2;

    lc_csw_volts.x1 = config.csw_calibration_x1;
    lc_csw_volts.y1 = config.csw_calibration_y1;
    lc_csw_volts.x2 = config.csw_calibration_x2;
    lc_csw_volts.y2 = config.csw_calibration_y2;

    config_calibration = config;
}

void Supervisor::set_config_periods(supervisor::SupervisorPeriodsConfig& config, unsigned int level)
{
    ROS_INFO_STREAM("Received new config periods data");

    timer_leak.setPeriod(ros::Duration(config.period_leak));
    timer_compensator.setPeriod(ros::Duration(config.period_compensator));
    timer_devices_status.setPeriod(ros::Duration(config.period_devices_status));
    timer_short_circuit.setPeriod(ros::Duration(config.period_short_circuit));
    timer_adc.setPeriod(ros::Duration(config.period_adc));
    timer_external_adc.setPeriod(ros::Duration(config.period_external_adc));
    timer_depth.setPeriod(ros::Duration(config.period_depth));

    timer_heartbeat.setPeriod(ros::Duration(config.period_heartbeat));
    timer_udp_parser.setPeriod(ros::Duration(config.period_parser));

    config_periods = config;
    
}

void Supervisor::set_config_devices(supervisor::SupervisorDevicesConfig& config, unsigned int level)
{
    ROS_INFO_STREAM("Received new config devices data");
    
    devices[static_cast<unsigned int>(SupervisorDevices::Grabber)].position   = config.grabber_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Grabber)].state      = config.grabber_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Dsp)].position       = config.DSP_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Dsp)].state          = config.DSP_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Cargo_1)].position   = config.cargo_1_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Cargo_1)].state      = config.cargo_1_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Cargo_2)].position   = config.cargo_2_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Cargo_2)].state      = config.cargo_2_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Torpedo_1)].position = config.torpedo_1_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Torpedo_1)].state    = config.torpedo_1_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Torpedo_2)].position = config.torpedo_2_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Torpedo_2)].state    = config.torpedo_2_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Dvl)].position       = config.dvl_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Dvl)].state          = config.dvl_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_1)].position = config.reserve_1_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_1)].state    = config.reserve_1_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_2)].position = config.reserve_2_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_2)].state    = config.reserve_2_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_3)].position = config.reserve_3_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_3)].state    = config.reserve_3_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_4)].position = config.reserve_4_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_4)].state    = config.reserve_4_state;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_5)].position = config.reserve_5_position;
    devices[static_cast<unsigned int>(SupervisorDevices::Reserve_5)].state    = config.reserve_5_state;

    unsigned short status = 0, mask = 0;
    for(auto &el: devices) {
        if(el.state) status += (1 << el.position);
    }
    
    mask = 0xFFFF;

    vector<unsigned char> data;
    data.push_back(static_cast<unsigned char>(status & 0xFF));
    data.push_back(static_cast<unsigned char>(status >> 8));
    data.push_back(static_cast<unsigned char>(mask & 0xFF));
    data.push_back(static_cast<unsigned char>(mask >> 8));

    marshall_and_publish(data, TcpHeaders::SwitchDevices);

    config_devices = config;
}

unsigned short Supervisor::from_bitmask(vector<unsigned char> data) 
{
    unsigned short value = 0;

    for (size_t i = 0; i < data.size(); ++i) {
        value += data[i] ? (1 << i) : 0;
    }

    return value;
}

void Supervisor::bitmask(vector<unsigned char>& data, vector<unsigned char> input_data_array)
{
    for(auto &el: input_data_array) {
        for (int i = 0; i < 8; i++) {
            data.push_back(static_cast<unsigned char>((el & (1 << i)) >> i));
        }
    }
}

void Supervisor::connection_heartbeat(const ros::TimerEvent& event)
{
    eth_connection->state_update();
}

void Supervisor::handle_message(const supervisor::CmdCan& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
    vector<unsigned char> data;
    
    data.push_back(msg.can_id);
    for (auto &el: msg.can_data) {
        data.push_back(el);
    }
    marshall_and_publish(data, TcpHeaders::Can);
}

void Supervisor::handle_message(const supervisor::CmdConfigureUdp& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
}

void Supervisor::handle_message(const supervisor::CmdDeviceKey& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));

    vector<unsigned char> data;

    unsigned short status = msg.state << devices[msg.id].position;
    unsigned short mask   = 1 << devices[msg.id].position;

    data.push_back(static_cast<unsigned char>(status & 0xFF));
    data.push_back(static_cast<unsigned char>(status >> 8));
    data.push_back(static_cast<unsigned char>(mask & 0xFF));
    data.push_back(static_cast<unsigned char>(mask >> 8));

    marshall_and_publish(data, TcpHeaders::SwitchDevices);
}

void Supervisor::handle_message(const supervisor::CmdFirmware& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
}

void Supervisor::handle_message(const supervisor::CmdPwm& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
}

void Supervisor::handle_message(const supervisor::CmdSystemFlags& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
}

void Supervisor::handle_message(const supervisor::CmdUart& msg)
{
    ROS_DEBUG_STREAM("Received " << ipc::classname(msg));
}

void Supervisor::marshall_and_publish(std::vector<unsigned char> data, TcpHeaders ID)
{
    vector<unsigned char> upload_data = data;
    upload_data.insert(upload_data.begin(), static_cast<char>(ID));
    upload_data.insert(upload_data.begin(), static_cast<char>(upload_data.size()));
    eth_connection->set_upload_data(upload_data);

    for (auto &el: upload_data) {
        cout << int(el) << ", ";
    }
    cout << endl;
}

void Supervisor::udp_data_parser(const ros::TimerEvent& event)
{
    vector<unsigned char> download_data;
    
    if(!is_simulating_) {
        eth_connection->get_download_data(download_data);
    } else {
        get_simulating_data(download_data);
        ROS_WARN_STREAM("Downloaded data size: " << download_data.size());
    }
    
    if (download_data.size() < 2) {
        ROS_WARN_STREAM("Downloaded data is empty");
        return;
    }

    unsigned char packet_size, header;
    vector<unsigned char> handed_data;

    for (int i = 0; i < download_data.size(); ) {
        packet_size = download_data[i];
        header = download_data[i + 1];
        handed_data.assign(download_data.begin() + i + 2, download_data.begin() + i + 2 + packet_size - 1);
        recognize_udp_header(static_cast<UdpHeaders> (header), handed_data);
        handed_data.clear();
        i += packet_size + 1;
    }
}

void Supervisor::get_simulating_data(std::vector<unsigned char>& data)
{
    //сначала размер, затем ID, затем данные

    //протечка
    data.push_back(2);
    data.push_back(static_cast<unsigned char>(UdpHeaders::Leak));
    data.push_back(0x1F);

    //состояние устройств
    data.push_back(3);
    data.push_back(static_cast<unsigned char>(UdpHeaders::Facilities));
    data.push_back(0x12);
    data.push_back(0x45);

    //компенсатор
    data.push_back(2);
    data.push_back(static_cast<unsigned char>(UdpHeaders::Compensator));
    data.push_back(0x17);

    //внутренний АЦП
    data.push_back(11*2 + 1);
    data.push_back(static_cast<unsigned char>(UdpHeaders::Adc));
    for(int i = 0; i < 11; i++) {
        short int test_value = 3000;
        data.push_back(static_cast<unsigned char>(test_value & 0xFF ));
        data.push_back(static_cast<unsigned char>(test_value >> 8));
    }

    data.push_back(6*4 + 1);
    data.push_back(static_cast<unsigned char>(UdpHeaders::AdcExternal));
    for(int i = 0; i < 6; i++) {
        unsigned int test_value = 10000000;
        data.push_back(static_cast<unsigned char>(test_value & 0xFF ));
        data.push_back(static_cast<unsigned char>((test_value >> 8) & 0xFF));
        data.push_back(static_cast<unsigned char>((test_value >> 16) & 0xFF));
        data.push_back(static_cast<unsigned char>((test_value >> 24) & 0xFF));
    }
}

void Supervisor::recognize_udp_header(UdpHeaders Id, std::vector<unsigned char> data)
{
    switch(Id) {
        case UdpHeaders::Leak:
            handler_udp_leak(data);
            break;
        case UdpHeaders::FirmwareReady:
            handler_udp_firmware_ready(data);
            break;
        case UdpHeaders::Facilities:
            handler_udp_devices_status(data);
            break;
        case UdpHeaders::ShortCircuit:
            handler_udp_short_circuit(data);
            break;
        case UdpHeaders::Adc:
            handler_udp_adc(data);
            break;
        case UdpHeaders::AdcExternal:
            handler_udp_external_adc(data);
            break;
        case UdpHeaders::Compensator:
            handler_udp_compensator(data);
            break;
        default:
            break;
    }
}

void Supervisor::handler_udp_leak (vector<unsigned char> data)
{
    uint leak_size = data.size() * 8;
    if (leak_size < 8) {
        ROS_WARN_STREAM("Packet leak is not full, size:" << data.size());
        return;
    }

    vector<unsigned char> leak_status;
    bitmask(leak_status, data);

    copy(leak_status.begin(), leak_status.end(), msg_leak.status.begin());
    msg_leak.header.stamp = ros::Time::now();
}

void Supervisor::handler_udp_firmware_ready (vector<unsigned char> data)
{

}

void Supervisor::handler_udp_devices_status (vector<unsigned char> data)
{
    
    uint device_size = data.size() * 8;
    
    if (device_size < 16) {
        ROS_WARN_STREAM("Packet devices_status is not full, size:" << data.size());
        return;
    }

    vector<unsigned char> devices_status;
    bitmask(devices_status, data);

    copy(devices_status.begin(), devices_status.end(), msg_devices_status.status.begin());
    msg_devices_status.header.stamp = ros::Time::now();
}

void Supervisor::handler_udp_short_circuit (vector<unsigned char> data)
{
    
    uint device_size = data.size() * 8;
    
    if (device_size < 16) {
        ROS_WARN_STREAM("Packet short_circuit is not full, size:" << data.size());
        return;
    }
    
    vector<unsigned char> sc_status;
    // unsigned short status = static_cast<unsigned short> (data[0]) +  
    //                         ((static_cast<unsigned short> (data[1]) ) << 8);
    bitmask(sc_status, data);

    copy(sc_status.begin(), sc_status.end(), msg_short_circuit.status.begin());
    msg_short_circuit.header.stamp = ros::Time::now();
}

void Supervisor::handler_udp_adc (vector<unsigned char> data)
{

    uint adc_size = data.size() / 2;
    
    if (adc_size < 11) {
        ROS_WARN_STREAM("Packet adc is not full, size:" << data.size());
        return;
    }

    double max_value_volt = 3.3;
    double max_value_dec = 1 << 12;
    vector<float> adc_values;

    unsigned short value_dec;
    for (int i = 0; i < adc_size; i += 2) {
        value_dec = *reinterpret_cast<unsigned short*>(data.data() + i);
        adc_values.push_back(max_value_volt * value_dec / max_value_dec);
    }

    copy(adc_values.begin(), adc_values.end(), msg_adc.values.begin());
    msg_adc.header.stamp = ros::Time::now();
}

void Supervisor::handler_udp_external_adc (vector<unsigned char> data)
{

    uint ext_adc_size = data.size() / 4;
    
    if (ext_adc_size < 6) {
        ROS_WARN_STREAM("Packet external_adc is not full, size:" << data.size());
        return;
    }

    double max_value_volt = 2.5;
    double max_value_dec = 1 << 24;
    vector<float> ext_adc_values;

    unsigned int value_dec;
    for (int i = 0; i < ext_adc_size; i++) {
        value_dec = *reinterpret_cast<unsigned int*>(data.data() + 4*i);
        ext_adc_values.push_back(max_value_volt * value_dec / max_value_dec);
    }

    copy(ext_adc_values.begin(), ext_adc_values.end(), msg_adc_ext.values.begin());
    msg_adc_ext.header.stamp = ros::Time::now();
}

void Supervisor::handler_udp_compensator (vector<unsigned char> data) {
    
    uint comp_size = data.size() * 8;
    
    if (comp_size < 8) {
        ROS_WARN_STREAM("Packet compensator is not full.");
        return;
    } else {
        ROS_WARN_STREAM("mes compensator ok, size:" << data.size());
    }

    vector<unsigned char> comp_status;
    unsigned short comp = data[0];
    //bitmask(comp_status, comp_size, comp);
    bitmask(comp_status, data);

    copy(comp_status.begin(), comp_status.end(), msg_comp.status.begin());
    msg_comp.header.stamp = ros::Time::now();
}


void Supervisor::publish_leak (const ros::TimerEvent& event)
{
    ROS_INFO_STREAM("Publish msg_leak");
    if (ros::Time::now().toSec() - ipc::timestamp(msg_leak) < config_periods.timeout_old_data) {
        leak_pub_.publish(msg_leak);    
    } else {
        ROS_WARN_STREAM("msg msg_leak is too old for publishing");
    }
}

void Supervisor::publish_compensator (const ros::TimerEvent& event)
{
    ROS_INFO_STREAM("Publish msg_compensator");
    if (ros::Time::now().toSec() - ipc::timestamp(msg_comp) < config_periods.timeout_old_data) {
        compensator_pub_.publish(msg_comp);
    } else {
        ROS_WARN_STREAM("msg msg_comp is too old for publishing");
    }
}

void Supervisor::publish_devices_status (const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_devices_status) < config_periods.timeout_old_data) {
        devices_status_pub_.publish(msg_devices_status);    
    } else {
        ROS_WARN_STREAM("msg msg_devices_status is too old for publishing");
    }
}

void Supervisor::publish_short_circuit (const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_short_circuit) < config_periods.timeout_old_data) {
        short_circuit_pub_.publish(msg_short_circuit);    
    } else {
        ROS_WARN_STREAM("msg msg_short_circuit is too old for publishing");
    }
}

void Supervisor::publish_adc (const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc) < config_periods.timeout_old_data) {
        adc_pub_.publish(msg_adc);    
    } else {
        ROS_WARN_STREAM("msg msg_adc is too old for publishing");
    }
}

void Supervisor::publish_external_adc (const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc_ext) < config_periods.timeout_old_data) {
        external_adc_pub_.publish(msg_adc_ext);    
    } else {
        ROS_WARN_STREAM("msg msg_adc_ext is too old for publishing");
    }
}

void Supervisor::publish_depth (const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc_ext) < config_periods.timeout_old_data) {
        msg_depth.depth = lc_depth.calibrate(msg_adc_ext.values[1]);
        depth_pub_.publish(msg_depth);    
    } else {
        ROS_WARN_STREAM("msg_adc_ext is too old for publishing depth");
    }

}

void Supervisor::publish_temperature(const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc) < config_periods.timeout_old_data) {
        msg_temperature.temperature = lc_temperature.calibrate(msg_adc.values[10]);
        temperature_pub_.publish(msg_temperature);    
    } else {
        ROS_WARN_STREAM("msg_adc is too old for publishing temperature");
    }
}

void Supervisor::publish_power_supply_state(const ros::TimerEvent& event)
{
    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc_ext) < config_periods.timeout_old_data) {
        msg_power_supply.battery_voltage = lc_battery_volts.calibrate(msg_adc_ext.values[4]);
        msg_power_supply.battery_current = lc_current.calibrate(msg_adc_ext.values[2]);
        power_supply_pub_.publish(msg_power_supply);    
    } else {
        ROS_WARN_STREAM("msg msg_adc_ext is too old for publishing supply_state");
    }
}

void Supervisor::water_check(const ros::TimerEvent& event)
{
    bool water_state;

    float lower_bounder = config_calibration.csw_level - 0.2;
    float upper_bounder = config_calibration.csw_level + 0.2;

    if (ros::Time::now().toSec() - ipc::timestamp(msg_adc) < config_periods.timeout_old_data) {
        float water_adc_level = lc_csw_volts.calibrate(msg_adc.values[8]);

        if(water_adc_level > upper_bounder) {
            water_state = 1;
        } else if (water_adc_level < lower_bounder) {
            water_state = 0;
        }
        msg_water_check.water_state = water_state;
        water_check_pub_.publish(msg_water_check);

    } else {
        ROS_WARN_STREAM("msg msg_adc is too old for publishing water_check");
    }
}
///@}