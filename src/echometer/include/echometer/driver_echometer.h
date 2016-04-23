
//==============================================================================
//
// Name        : driver_echometer.h
// Author      : Anton Tolstonogov
// Description : Подпрограммы для работы с эхолотом
// Modified    : 2015-08-00 18-00
//
//==============================================================================

#ifndef DRIVER_ECHOMETER_H
#define DRIVER_ECHOMETER_H

//------------------------------------------
//Includes
#include "echometer/com_port.h"    // Подпрограммы для работы с COM-портом
#include "echometer/nmea_parser.h" // Подпрограммы для работы с парсингом строк в формате NMEA
#include <time.h>
#include <sys/timeb.h>
#include <errno.h>
#include <pthread.h>
//------------------------------------------
//Defines
#define POWER_ON_WAIT   (1000)
#define DEVICE_WAIT     (200)

//------------------------------------------
//Enums
typedef enum {
    ALTIMETER_SIMPLE    = 1,
    ECHOSOUNDER         = 2,
    ALTIMETER_NMEA      = 3,
    ECHOSOUNDER_200     = 4,
    ECHOSOUNDER_500     = 5,
    ECHOSOUNDER_1000    = 6
}Workmode;

typedef enum {
    INTERNAL = 0,
    EXTERNAL = 1
}Syncmode;

typedef enum {
    FALLING = 0,
    RISING  = 1
}Syncedge;

typedef struct {
    double depth;
    double depth_below;
    double depth_offset;
    double depth_max;
    double temperature;
    double time;
    bool new_data;
    bool bad_data;
} echosouder_data_struct;

typedef struct
{
    Workmode workm;
    int range;
    int threshold;
    double interval;
    int level;
    int offset;
    int deadzone;
    int txlength;
    Syncmode syncextern;
    Syncedge syncextmod;
} echosouder_config_struct;

//Настройки драйвера
bool open_echosounder (const char *COM_name, int baudrate);
bool close_echosounder (void);

bool start_listen (void);

bool stop_listen (void);

// Вывод заголовка программы
void print_program_header (void);

// Получить данные от Эхолота
bool get_data(echosouder_data_struct& data);

//получить текущие настройки эхолота
bool get_config(echosouder_config_struct& config);

// Включение эхолота
void echosounder_start (void);

// Выключение эхолота
void echosounder_stop (void);

// Перезапуск эхолота
void echosounder_reset (void);

//Сконфигурировать датчик
void echosounder_set_config(echosouder_config_struct config);

//Отправить команду
void send_command(char* comand, char* val);

//Установть рабочий режим
void set_workmode(Workmode workm);

//Установить диапазон работы, mm [1000; 100000]
void set_range(int range_mm);

//Установить интервал пингов, sec [0.1; 3600]
void set_interval(double interval_sec);

//Установить уровень детектирования сигнала, %% [0; 10000]
void set_threshold(int level);

//Установить смещение, mm
void set_offset(int offset_mm);

//Установить мертвую зону, mm
void set_deadzone(int level_mm);

//Установить величину импульса, mcs
void set_txlength(int length_mcs);

//Установить режим синхронизации
void set_syncmode(Syncmode mode);

//Установить синхронизацию по возврастающей или спадающей волне
void set_syncedge(Syncedge mode);

//Ожидание (мсек)
void wait (int delay_msec);

//Callback функция обработки данных с com-порта
//void data_handler(int);
#endif // DRIVER_ECHOMETER_H