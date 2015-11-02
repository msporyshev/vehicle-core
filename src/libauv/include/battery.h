/*
Все структуры данных и драйвер написаны из расчета на три батарейки
для совместимости со старым протоколом и на всякий сулчай.
Батареек на самом деле две.
*/

#ifndef BATTERY_H
#define BATTERY_H

#define BATTERY_CAN_ADDR 400

//int и unsigned int здесь имеются ввиду скорее всего двухбайтовые
//потому что посылки должны влазить в 8 байт

//BATTERY_CAN_ADDR + 0 - ящик для приёма команд
typedef enum {
    Crash_mode_cmd = 1,
    Sleep_mode_cmd,
    Recharge_mode_cmd,
    Work_mode_cmd,
    ShutDown_mode_cmd,
    Charge_bat_cmd,
    Key_bat_cmd,
    Key_light_cmd,
    Key_system_cmd,
    Water_test_cmd
} BATTERY_COMMAND;

#define command_mask 0x0F
#define on_off_mask 0x80
#define N_offset 4
#define N_mask 0x07

//BATTERY_CAN_ADDR + 1
typedef struct {
    unsigned char VDC_C_flag:1;
    unsigned char VDC_ON_flag:1;
    unsigned char Crash_flag:1;
    unsigned char Sleep_flag:1;
    unsigned char Recharge_flag:1;
    unsigned char Work_flag:1;
    unsigned char ShutDown_flag:1;
    unsigned char Command_err:1;
} STATE_BYTE_1; 

typedef struct {
    unsigned char System_key_flag:1;
    unsigned char Light_key_flag:1;
    unsigned char bat1_key_flag:1;
    unsigned char bat2_key_flag:1;
    unsigned char bat3_key_flag:1;
    unsigned char bat_I_limit:1;
    unsigned char bat_disconnect:1;
    unsigned char Water_sensor:1;
} STATE_BYTE_2;

typedef struct {
    unsigned char Light_fuze_flag:1;
    unsigned char System_fuze_flag:1;
    unsigned char bud1_fuze_flag:1;
    unsigned char bud2_fuze_flag:1;
    unsigned char bud3_fuze_flag:1;
    unsigned char bud4_fuze_flag:1;
    unsigned char bud5_fuze_flag:1;
    unsigned char Water_test:1;
} STATE_BYTE_3;

typedef struct STATE_BYTE_4 {
    unsigned char bat1_Hi_Temp:1;
    unsigned char bat1_key_Hi_Temp:1;
    unsigned char bat1_Low_volt:1;
    unsigned char bat1_recharge_compl:1;
    unsigned char bat1_recharge:1;
    unsigned char bat1_recharge_01c:1;
    unsigned char bat1_no_conect_PS:1;
    unsigned char bat1_no_recharge_curent:1;
} STATE_BYTE_4;

typedef struct STATE_BYTE_5 {
    unsigned char bat2_Hi_Temp:1;
    unsigned char bat2_key_Hi_Temp:1;
    unsigned char bat2_Low_volt:1;
    unsigned char bat2_recharge_compl:1;
    unsigned char bat2_recharge:1;
    unsigned char bat2unsigned_recharge_01c:1;
    unsigned char bat2_no_conect_PS:1;
    unsigned char bat2_no_recharge_curent:1;
} STATE_BYTE_5;

typedef struct STATE_BYTE_6 {
    unsigned char bat3_Hi_Temp:1;
    unsigned char bat3_key_Hi_Temp:1;
    unsigned char bat3_Low_volt:1;
    unsigned char bat3_recharge_compl:1;
    unsigned char bat3_recharge:1;
    unsigned char bat3_recharge_01c:1;
    unsigned char bat3_no_conect_PS:1;
    unsigned char bat3_no_recharge_curent:1;
} STATE_BYTE_6;

//BATTERY_CAN_ADDR + 2
typedef struct
{
    float current_all;
    float voltage;
    float discharged;
} work_data1;

//BATTERY_CAN_ADDR + 3
typedef struct
{
    float bat2_voltage;
    float bat1_voltage;
    float bat3_voltage;
} work_data2;

//BATTERY_CAN_ADDR + 4
typedef struct
{
    unsigned char bat3_key_temp;
    unsigned char bat2_key_temp;
    unsigned char bat1_key_temp;
    unsigned char bat1_temp;
    unsigned char bat2_temp;
    unsigned char bat3_temp;
} work_data3;

//BATTERY_CAN_ADDR + 5
typedef struct
{
    float PS1_current;
    float PS1_voltage;
} charge_data1;

//BATTERY_CAN_ADDR + 6
typedef struct
{
    float PS2_current;
    float PS2_voltage;
} charge_data2;

//BATTERY_CAN_ADDR + 7
typedef struct
{
    float PS3_current;
    float PS3_voltage;
} charge_data3;

//BATTERY_CAN_ADDR + 8
typedef struct
{
    float charge_bat1;
    float charge_bat2;
    float charge_bat3;
} charge_data4;

#endif //BATTERY_H
