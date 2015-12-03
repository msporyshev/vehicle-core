#include "compass/settings.h"

using namespace std;

std::vector<struct_sensor> device_settings;

uint8_t* reverse_array(uint8_t* data, uint8_t size)
{
  for(uint8_t i = 0; i < size / 2; i++) {
    uint8_t temp = data[i];
    data[i] = data[size - i - 1];
    data[size - i - 1] = temp;
  }
  return data;
}

void callback_set_param(inemo_frame_struct frame)
{
}

void callback_get_param(inemo_frame_struct frame)
{
    int sensor_id = frame.payload[0];
    int param_id = frame.payload[1];

    cout << "New callback_get_param with sensor_id and param_id ";
    cout << sensor_id << ", " << param_id << "." << endl;

    for (auto &sensor: device_settings) {
        if(sensor_id == sensor.id) {
            for(auto &parameter: sensor.parameter_list) {
                if(param_id == parameter.id) {
                    if(parameter.type == "enum") {
                        parameter.value = frame.payload[2];
                    } else if (parameter.type == "numeric") {
                            parameter.value = *(short int*)&(frame.payload[2]);
                            reverse_array((uint8_t*)&(parameter.value), 2);
                    }
                }
            }
        }
    }
}


Compass_settings::Compass_settings()
{

}

Compass_settings::~Compass_settings()
{

}

void operator>> (const YAML::Node& node, struct_parameter& parameter) 
{
    YamlReader parameter_cfg(node);
    parameter_cfg.read_param(parameter.id, "parameter_id");
    parameter_cfg.read_param(parameter.name, "name");
    parameter_cfg.read_param(parameter.type, "type");
    parameter_cfg.read_param(parameter.value, "value");
}

void operator>> (const YAML::Node& node, struct_sensor& sensor) 
{
    YamlReader sensor_cfg(node);
    sensor_cfg.read_param(sensor.id, "id");
    sensor_cfg.read_param(sensor.name, "name");

    YAML::Node param_configs;
    sensor_cfg.read_param(param_configs, "parameters");

    for(size_t i = 0; i < param_configs.size(); i++) {
        struct_parameter parameter;
        param_configs[i] >> parameter;
        sensor.parameter_list.push_back(parameter);
    }
}

bool Compass_settings::load_config(YamlReader& cfg)
{
    YAML::Node sensors_configs;
    cfg.read_param(sensors_configs, "sensors");

    for(size_t i = 0; i < sensors_configs.size(); i++) {
        struct_sensor sensor;
        sensors_configs[i] >> sensor;
        config_data.push_back(sensor);
    }

    for (size_t i = 0; i < config_data.size(); ++i) {
        cout << "Sensor id: " << config_data[i].id << endl;
        cout << "Sensor name: " << config_data[i].name << endl;
        cout << "Parameters:" << endl;
        for(size_t j = 0; j < config_data[i].parameter_list.size(); j++) {
            cout << "Parameter id: " << config_data[i].parameter_list[j].id << endl;
            cout << "Parameter name: " << config_data[i].parameter_list[j].name << endl;
            cout << "Parameter value: " << config_data[i].parameter_list[j].value << endl;
            cout << "Parameter type: " << config_data[i].parameter_list[j].type << endl;
        }
    }
    return 0;
}

bool Compass_settings::get_all_settings()
{

    device_settings = config_data;
    if(device_settings.size() == 0) {
        cout << "config file not inizialized" << endl;
        return 1;
    }

    registrate_data_callback(INEMO_GET_SENSOR_PARAMETER, callback_get_param);
    
    for (auto &sensor: device_settings) {
        for (auto &parameter: sensor.parameter_list) {
            parameter.value = 0;
        }
        get_sensor_settings(sensor);
    }

    for (auto &sensor: device_settings) {
        cout << "Sensor id: " << sensor.id << endl;
        cout << "Sensor name: " << sensor.name << endl;
        cout << "Parameters:" << endl;
        for(auto &parameter: sensor.parameter_list) {
            cout << "\tParameter id: " << parameter.id << endl;
            cout << "\tParameter name: " << parameter.name << endl;
            cout << "\tParameter value: " << parameter.value << endl;
            cout << "\tParameter type: " << parameter.type << endl;
        }
    }

    deregistrate_data_callback(INEMO_SET_SENSOR_PARAMETER);
    return 0;
}

bool Compass_settings::set_all_settings()
{
    if(config_data.size() == 0) {
        cout << "config file not inizialized" << endl;
        return 1;
    }

    for (auto &sensor: config_data) {
        set_sensor_settings(sensor);
    }

}

bool Compass_settings::get_sensor_settings(struct_sensor& sensor)
{
    bool status;
    for (auto &parameter: sensor.parameter_list) {
        status = get_sensor_param(sensor.id, parameter);
        if(status != 0) {
            cout << "Get sensor parameter for " << sensor.name << " with " << \
            "parameter " << parameter.name << " going wrong." << endl;
            return 1;
        }
    }
    return 0;
}

bool Compass_settings::set_sensor_settings(struct_sensor sensor)
{
    bool status;
    for (auto &parameter: sensor.parameter_list) {
        status = set_sensor_param(sensor.id, parameter);
        if(status != 0) {
            cout << "Set sensor parameter for " << sensor.name << " with " << \
            "parameter " << parameter.name << " going wrong." << endl;
            return 1;
        }
    }
    return 0;
}

bool Compass_settings::get_sensor_param(int sensor_id, struct_parameter& parameter)
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 3;
    command_frame.id = INEMO_GET_SENSOR_PARAMETER;
    command_frame.payload[0] = static_cast<unsigned char>(sensor_id);
    command_frame.payload[1] = static_cast<unsigned char>(parameter.id); 

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool Compass_settings::set_sensor_param(int sensor_id, struct_parameter parameter)
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.id = INEMO_SET_SENSOR_PARAMETER;
    command_frame.payload[0] = static_cast<unsigned char>(sensor_id);
    command_frame.payload[1] = static_cast<unsigned char>(parameter.id); 
    if(parameter.type == "enum") {
        command_frame.length = 4;
        command_frame.payload[2] = parameter.value;
    } else if(parameter.type == "numeric") {
        command_frame.length = 5;
        memcpy((void*)&(command_frame.payload[2]), (void*)&(parameter.value), sizeof(short int));
        reverse_array((uint8_t*)&(command_frame.payload[2]), sizeof(short int));
    }

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
    return 0;
}

bool Compass_settings::save_settings_flash()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_SAVE_TO_FLASH;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool Compass_settings::load_settings_flash()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_LOAD_FROM_FLASH;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool Compass_settings::restore_settings()
{
    return 0;
}