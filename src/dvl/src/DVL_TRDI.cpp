#include "dvl/DVL_TRDI.h"
#include <iostream>
#include <termios.h> /* Объявления управления POSIX-терминалом */
#include <stdio.h>   /* Стандартные объявления ввода/вывода */
#include <unistd.h>  /* Объявления стандартных функций UNIX */
#include <fcntl.h>   /* Объявления управления файлами */
#include <errno.h>   /* Объявления кодов ошибок */
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>

using namespace std;

#define COM_BAUDRATE B115200
#define DEBUG        0

DvlTrdiDriver::DvlTrdiDriver()
{
    invalid_port_handle = -1;
    port_handle = invalid_port_handle;
}

bool DvlTrdiDriver::dvl_start(const char* com_port, int baudrate)
{
    // Определение скорости обмена
    switch (baudrate) {
    case 300:          baudrate = B300; break;
    case 1200:          baudrate = B1200; break;
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
        close (port_handle);
        return 1;
    }

    if (port_handle != invalid_port_handle)
        close(port_handle);

    port_handle = open(com_port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (port_handle == invalid_port_handle) {
        return 1;
    }

    // Настраивем порт
    struct termios options;

    tcgetattr(port_handle , &options);

    // !!! Частота посылок Доплера
    cfsetspeed(&options, baudrate);

    options.c_cflag |= ( CLOCAL | CREAD );

    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    options.c_cflag &= ~CRTSCTS;    // деактивируем аппаратное управление потоком

    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INPCK | ISTRIP);
    options.c_iflag |= (INPCK | ISTRIP);
    options.c_iflag |= IGNPAR;

    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tcsetattr(port_handle, TCSANOW, &options);

    return 0;
}

bool DvlTrdiDriver::dvl_stop()
{
    if (close(port_handle)) {
        return 1;
    } else {
        port_handle = invalid_port_handle;
        return 0;
    }
}

bool DvlTrdiDriver::decoding_data(char* list)
{
    if((strncmp(list,":BI,", 4)) == 0)
    {
        sscanf(list, ":BI, %d, %d, %d, %d, %c\n", &irefv_bottom.forward, &irefv_bottom.right,
            &irefv_bottom.down, &irefv_bottom.error, &irefv_bottom.status);
        irefv_bottom.new_data = 1;
        return 0;
    }else if((strncmp(list,":BD,", 4)) == 0)
    {
        sscanf(list, ":BD,  %f, %f, %f, %f, %f", &erefd_bottom.east, &erefd_bottom.north, &erefd_bottom.upward,
            &erefd_bottom.range, &erefd_bottom.time);
        erefd_bottom.new_data = 1;
        return 0;
    }else if((strncmp(list,":WI,", 4)) == 0)
    {
        sscanf(list, ":WI, %d, %d, %d, %d, %c", &irefv_water.forward, &irefv_water.right, &irefv_water.down,
            &irefv_water.error, &irefv_water.status);
        irefv_water.new_data = 1;
        return 0;
    }

    return -1;
}

bool DvlTrdiDriver::dvl_process()
{
    return dvl_read();
}

bool DvlTrdiDriver::dvl_read()
{
    static int i = 0;
    char c;
    int status;

    while (read(port_handle, &c, 1) == 1) {
        // Если конец строки, то переходим к расшифровке
        if (c == 10) {
            buffer[i] = 0;

            status = decoding_data(buffer);

            i = 0;
            return (status);
        } else {
            // Заполнение буфера принятыми символами
            buffer[i++] = c;
        }

        // Защита от переполнения буфера
        if (i >= READ_BUF_SUZE) {
            i = 0;
            return 1;
        }
    }

    return 1;
}

bool DvlTrdiDriver::get_instrument_velocity(instrument_reference_vel& data, reference_type type)
{
    if(type == REF_TYPE_WATERMASS) {
        if(irefv_water.new_data && irefv_water.status == 'A') {
            data = irefv_water;
            irefv_water.new_data = 0;
            return 0;
        }
    } else {
        if(irefv_bottom.new_data && irefv_bottom.status == 'A') {
            data = irefv_bottom;
            irefv_bottom.new_data = 0;
            return 0;
        }
    }
    return 1;
}

bool DvlTrdiDriver::get_ship_velocity(ship_reference_vel& data, reference_type type)
{
    if(type == REF_TYPE_WATERMASS) {
        if(srefv_water.new_data ) {
            data = srefv_water;
            srefv_water.new_data = 0;
            return 0;
        }
    } else {
        if(srefv_bottom.new_data) {
            data = srefv_bottom;
            srefv_bottom.new_data = 0;
            return 0;
        }
    }
    return 1;
}

bool DvlTrdiDriver::get_earth_velocity(earth_reference_vel& data, reference_type type)
{
    if(type == REF_TYPE_WATERMASS) {
        if(erefv_water.new_data) {
            data = erefv_water;
            erefv_water.new_data = 0;
            return 0;
        }
    } else {
        if(erefv_bottom.new_data) {
            data = erefv_bottom;
            erefv_bottom.new_data = 0;
            return 0;
        }
    }
    return 1;
}

bool DvlTrdiDriver::get_earth_distance(earth_reference_dist& data, reference_type type)
{
    if(type == REF_TYPE_WATERMASS) {
        if(erefd_water.new_data) {
            data = erefd_water;
            erefd_water.new_data = 0;
            return 0;
        }
    } else {
        if(erefd_bottom.new_data) {
            data = erefd_bottom;
            erefd_bottom.new_data = 0;
            return 0;
        }
    }
    return 1;
}