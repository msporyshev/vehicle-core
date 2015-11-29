/**
\file
\brief Драйвер компаса

В данном файле находятся все фукции получения данных от компаса, их
интерпретации и подготовки физических величин.

\ingroup compass_node

*/

#pragma once

//------------------------------------------
#include "compass/packet_handler.h"
#include <errno.h>
#include <cstdint>

//------------------------------------------
//Defines
#define POWER_ON_WAIT	(1000)
#define DEVICE_WAIT		(200)

//------------------------------------------

#pragma pack(1)

typedef struct Component_s
{
  short X;
  short Y;
  short Z;
} Component_t;

typedef struct Rotation_s
{
  float Roll;
  float Pitch;
  float Yaw;
}Rotation_t;

typedef struct Quaternion_s
{
  float Q1;
  float Q2;
  float Q3;
  float Q4;
}Quaternion_t;

typedef struct Compass_s
{
  float Roll;
  float Pitch;
  float Heading;
}Compass_t;

#pragma pack()

/*******************************************************************************
* The INEMO_M1_ID_t are the command ID for the device iNEMO
*******************************************************************************/



typedef enum {
  SAMPLE_NUM,
  ACCELEROMETER,
  GYROSCOPE,
  MAGNETOMETER,
  PRESSURE,
  TEMPERATURE,
  ROTATION,
  QUATERNION,
  COMPASS,
  SENSORS_NUM
} SENSORS;

typedef struct sensor_struct_e
{
  uint8_t id;
  uint8_t size;
  uint8_t sensor_status;
  uint8_t data_status;
  void*   pointer;
}sensor_struct_t;

typedef struct 
{
  uint8_t status;
} compass_config_struct;

bool open_compass(const char *com_name, int baudrate);

bool get_comport_status();

bool close_compass(void);

bool start_listen(void);

bool stop_listen(void);

void set_debug_level(uint8_t level);

void set_compass_config(compass_config_struct conf);

bool start_calibration(int timeout);

compass_config_struct get_compass_config();

bool INEMO_M1_Connect();

bool INEMO_M1_Disconnect();

bool INEMO_M1_Config_Output();

bool INEMO_M1_Start_Acquisition();

bool INEMO_M1_Stop_Acquisition();

bool INEMO_M1_Start_Calibration();

bool INEMO_M1_Abort_Calibration();


uint8_t Get_Counter(uint32_t* data);

uint8_t Get_Rotation_Data(Rotation_t* data);

uint8_t Get_Quaternion_Data(Quaternion_t* data);

uint8_t Get_Compass_Data(Compass_t* data);

uint8_t Get_Accelerometer_Data(Component_t* data);

uint8_t Get_Gyroscope_Data(Component_t* data);

uint8_t Get_Magnetometer_Data(Component_t* data);

uint8_t Get_Pressure_Data(uint32_t* data);

uint8_t Get_Temperature_Data(uint16_t* data);

//Ожидание (мсек)
void wait (int delay_msec);
