//============================================================//
//                                                            //
//       Работа с инерциальным измерительным модулем          //
//              MTi (Xsens Technologies B.V.)                 //
//                                                            //
//============================================================//

#pragma once

#include <stdio.h>              // Standard Input / Output
#include <stdlib.h>             // General Utilities
#include <string.h>             // String Handling
#include <unistd.h>             // Symbolic Constants
#include <fcntl.h>              // File Control Operations
#include <termios.h>            // General Terminal Interface
#include <sys/ioctl.h>          //
#include <stdarg.h>     /* va_list, va_start, va_arg, va_end */

#include "ros/ros.h"
//------------------------------------------------------------------------------
// Тип, описывающий структуру с полученными от устройства данными измерений
struct MTI_struct
{
    float temp; 
    float accX, accY, accZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float roll, pitch, yaw;
    unsigned short int Ain_1, Ain_2;
    unsigned char status;
    unsigned short int TS;
};

class MTI
{
public:
    MTI();

    // Плохой файловый дескриптор
    int invalid_file_descriptor;
    //------------------------------------------------------------------------------
    // Команды управления устройством
    static unsigned char GoToConfig[];
    static int GoToConfig_size;
    static unsigned char GoToMeasurement[];
    static int GoToMeasurement_size;
    static unsigned char Reset[];
    static int Reset_size;
    static unsigned char SetBaudrate57600[];
    static int SetBaudrate57600_size;
    static unsigned char SetBaudrate38400[];
    static int SetBaudrate38400_size;
    static unsigned char SetMeasurementMode[];
    static int SetMeasurementMode_size;
    static unsigned char SetMeasurementSettings[];
    static int SetMeasurementSettings_size;
    static unsigned char SetRawMode[];
    static int SetRawMode_size;
    static unsigned char SetRawSettings[];
    static int SetRawSettings_size;
    static unsigned char StoreXKFState[];
    static int StoreXKFState_size;

    int show_debug_messages;

    void mprintf (const char *format, ...);

    // Вывод в STDOUT в шестнадцатеричном представлении
    // содержимого num ячеек памяти, начиная с адреса addr
    void print_memory_table (unsigned char *addr, int num);

    // Вывод в STDOUT в шестнадцатеричном представлении в виде таблицы
    // содержимого num ячеек памяти, начиная с адреса addr
    void print_memory_full_table (unsigned char *addr, int num);

    //------------------------------------------------------------------------------
    // Открытие COM-порта и его настройка для работы с инерциальным
    // измерительным модулем MTi (Xsens Technologies B.V.)
    int MTI_COM_open (const char *COM_name, int baudrate);

    //------------------------------------------------------------------------------
    // Закрытие COM-порта
    void MTI_COM_close (int *fd);

    //------------------------------------------------------------------------------
    // Расшифровка полученного пакета данных
    // Возвращает 0 в случае успешной расшифровки пакета
    // Возвращает -1 в случае возникновения ошибки (неверная длина пакета)
    int package_decode (unsigned char *package, MTI_struct *MTI_data_pointer);

    //------------------------------------------------------------------------------
    // ВНИМАНИЕ!!! ОПАСНАЯ ФУНКЦИЯ!!!
    // Блокирующее чтение в буфер блока данных с COM-порта
    // Всегда возвращает 0
    int read_buffer_block (int fd, unsigned char* buffer, int len);

    //------------------------------------------------------------------------------
    // Неблокирующее чтение в буфер блока данных с COM-порта
    // Возвращает  0 в случае успешного чтения
    // Возвращает -1 в случае возникновения ошибки (в том числе в случае частичного чтения)
    int read_buffer_nonblock (int fd, unsigned char* buffer, int len);

    //------------------------------------------------------------------------------
    // Неблокирующее чтение в буфер блока данных с COM-порта
    // Возвращает число реально прочитанных байт
    // или -1 в случае возникновения ошибки
    int read_buffer_part_nonblock (int fd, unsigned char *buffer, int size);

    //------------------------------------------------------------------------------
    // ВНИМАНИЕ!!! ОПАСНАЯ ФУНКЦИЯ!!!
    // Блокирующее чтение 1 пекета данных с компаса
    int MTI_get_one_package_block (int fd, MTI_struct *MTI_data_pointer);

    //------------------------------------------------------------------------------
    // Неблокирующее чтение 1 пекета данных с компаса
    // Возвращает 0 в случае успешного чтения
    // или -1 в случае возникновения ошибки
    int MTI_get_one_package_nonblock (int fd);

    //------------------------------------------------------------------------------
    // Передача пекета данных на компас
    int MTI_send_package (int fd, const unsigned char *buffer, int len);

    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------
    // Чтение пекета данных с компаса
    int MTI_get_package (int fd, unsigned char **buffer, int *len);

    //------------------------------------------------------------------------------
    // Расчет контрольной суммы
    unsigned char MTI_ComputeChecksum (unsigned char *buffer, int len);

    //==============================================================================
    //
    // Отправка компасу команд управления
    //
    //==============================================================================
    
    void MTI_reset (int fd);

    void MTI_init (int fd);

    void MTI_store_filter (int fd);

    void MTI_start_calibrate (int fd);

    void MTI_stop_calibrate (int fd);
};