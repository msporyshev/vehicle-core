/**
\file
\brief Настройка конфигурации компаса

В данном файле находятся все фукции для конфигурации компаса

\ingroup compass_node

*/

///@{

#pragma once

#include <stdio.h>              // Standard Input / Output
#include <string>
#include <vector>
#include <map>
#include <iostream>

#include "yaml_reader.h"
#include "compass/packet_handler.h"

typedef struct
{
	int 			id;
	std::string 	name;
	short int		value;
	std::string 	type;
} struct_parameter;

///Структура сенсора на датчике компаса
typedef struct 
{
	int 							id; //< ID сенсора
	std::string 					name; //< имя сенсора
	std::vector<struct_parameter> 	parameter_list; //< список параметров сенсора
} struct_sensor;

/**
	\brief Класс настроек компаса
*/
class Compass_settings
{
public:
	
	Compass_settings();

	~Compass_settings();

	bool load_config(YamlReader& cfg);

	bool get_all_settings();

	bool set_all_settings();

	bool get_sensor_settings(struct_sensor& sensor);
	
	bool set_sensor_settings(struct_sensor sensor);
	
	bool get_sensor_param(int sensor_id, struct_parameter& parameter);
	
	bool set_sensor_param(int sensor_id, struct_parameter parameter);

	// void callback_get_param(inemo_frame_struct frame);

	// void callback_set_param(inemo_frame_struct frame);
	
	bool save_settings_flash();

	bool load_settings_flash();	

	bool restore_settings();

private:

	YamlReader cfg;

	std::vector<struct_sensor> config_data;
};

///@}