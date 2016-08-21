/*********************************************************************
 *
 * Name        : COM-ports.h
 * Author      : Fedor Dubrovin
 * Description : Подпрограммы для работы с COM-портом
 * Version     : 001
 * Modified    : 2010_12_16 13-00
 *
 ********************************************************************/

#ifndef COMPORTS_H_
#define COMPORTS_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


// Открытие COM-порта
int COM_open(const char *filename)
{
	int fd;

	// Открытие порта
	fd = open(filename, O_RDWR | O_NOCTTY | O_NDELAY);

	// Результат открытия порта
	if(fd == -1)
		printf("\nПорт %s не найден.\n", filename);
	else
		printf("\nПорт %s успешно открыт.\n", filename);

	return (fd);
}


// Закрытие COM-порта
int COM_close(int fd)
{
	return ( close(fd) );
}


// Настройка COM-порта
int COM_settings(int fd, int speed)
{
	struct termios options;

	// Определение скорости обмена
	speed_t baudrate;
    switch (speed)
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
        return -1;
    }

	if ( tcgetattr(fd , &options) != 0 )
		return (-1);

	if ( cfsetispeed(&options, baudrate) != 0 )
		return (-1);

	if ( cfsetospeed(&options, baudrate) != 0 )
		return (-1);

	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;

	options.c_cflag &= ~CRTSCTS;	// деактивируем аппаратное управление потоком

	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	options.c_iflag &= ~(INLCR | ICRNL | IGNCR | IUCLC | IMAXBEL | BRKINT);

	options.c_oflag &= ~OPOST;

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	if ( tcsetattr(fd , TCSANOW, &options) != 0 )
		return (-1);

	return (0);
}


#endif /* COMPORTS_H_ */
