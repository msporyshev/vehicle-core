//==============================================================================
//
// Работа с COM-портом (открытие, настройка, закрытие)
//
//==============================================================================
#pragma once

#include <fcntl.h>              // File Control Operations
#include <termios.h>            // General Terminal Interface

#include <ros/ros.h>


//------------------------------------------------------------------------------

// Плохой файловый дескриптор
#define INVALID_FILE_DESCRIPTOR         -1

//------------------------------------------------------------------------------
// Открытие COM-порта и его настройка
//------------------------------------------------------------------------------
int COM_open(const char *COM_name, int baudrate)
{
    int fd;

    fd = open(COM_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == INVALID_FILE_DESCRIPTOR) {
        ROS_ERROR_STREAM("Can not open " << COM_name << " port");
        return INVALID_FILE_DESCRIPTOR;
    } else {
        ROS_INFO_STREAM("Port " << COM_name << " was successfully opened");
    }


    struct termios options;       // Структура с настройками COM-порта

    // Получение текущих настроек COM-порта
    if (tcgetattr(fd , &options) != 0) {
        ROS_ERROR("Unable to get the current settings of COM-port. COM-port was closed.");
        close(fd);
        return INVALID_FILE_DESCRIPTOR;
    }

    // Определение скорости обмена
    switch (baudrate)
    {
        case 2400:          baudrate = B2400; break;
        case 4800:          baudrate = B4800; break;
        case 9600:          baudrate = B9600; break;
        case 19200:         baudrate = B19200; break;
        case 38400:         baudrate = B38400; break;
        case 57600:         baudrate = B57600; break;
        case 115200:        baudrate = B115200; break;
        case 230400:        baudrate = B230400; break;
        case 460800:        baudrate = B460800; break;
        default:
        ROS_ERROR("Incorrect COM-port baudrate. COM-port was closed.");
        close(fd);
        return INVALID_FILE_DESCRIPTOR;
    }

    // Установка скорости работы COM-порта
    if (cfsetspeed(&options, baudrate) != 0) {
        ROS_ERROR("Unable to set COM-port baudrate. COM-port was closed." );
        close(fd);
        return INVALID_FILE_DESCRIPTOR;
    }

    // Управляющие опции (c_cflag)

    options.c_cflag |= ( CLOCAL | CREAD );        // Обязательные опции

    options.c_cflag &= ~CSIZE;    // Маскирование битов размера симвоов
    //options.c_cflag |= CS5;       // 5 бит данных
    //options.c_cflag |= CS6;       // 6 бит данных
    //options.c_cflag |= CS7;       // 7 бит данных
    options.c_cflag |= CS8;       // 8 бит данных

    //options.c_cflag |= PARENB;    // Использовать бит четности
    options.c_cflag &= ~PARENB;   // Не использовать бит четности

    //options.c_cflag |= PARODD;    // Проверка на нечетность
    //options.c_cflag &= ~PARODD;   // Проверка на четность

    //options.c_cflag |= CSTOPB;    // Два стоповых бита
    options.c_cflag &= ~CSTOPB;   // Один стоповый бит

    //options.c_cflag |= CRTSCTS;   // Включение аппаратного управления потоком
    options.c_cflag &= ~CRTSCTS;  // Отключение аппаратного управления потоком

    // Опции ввода (c_iflag)

    //options.c_iflag |= ( IXON | IXOFF | IXANY );  // Включение программного управления потоком
    options.c_iflag &= ~( IXON | IXOFF | IXANY ); // Отключение программного управления потоком

    options.c_iflag &= ~( INLCR | ICRNL | IGNCR | IUCLC | IMAXBEL | BRKINT );

    // Опции вывода (c_oflag)

    //options.c_oflag |= OPOST;    // Режим обработанного вывода
    options.c_oflag &= ~OPOST;    // Режим необработанного вывода. Остальные биты поля c_oflag игнорируются.

    // Локальные опции (c_lflag)

    options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

    // Применение данных настроек COM-порта
    if(tcsetattr(fd, TCSANOW, &options) != 0) {
        ROS_ERROR("Unable to set COM-port. COM-port was closed." );
        close(fd);
        return INVALID_FILE_DESCRIPTOR;
    }

    return fd;
}


//------------------------------------------------------------------------------
// Закрытие COM-порта
//------------------------------------------------------------------------------
void COM_close(int *fd)
{
    if ( *fd != INVALID_FILE_DESCRIPTOR ) {
        ROS_INFO("COM-port was closed" );
        close(*fd);
        *fd = INVALID_FILE_DESCRIPTOR;
    } else {
        ROS_ERROR("Can not close COM-port. Perhaps COM-port is already closed");
    }
}
