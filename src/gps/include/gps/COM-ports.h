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
int COM_settings(int fd, speed_t speed)
{
	struct termios options;

	if ( tcgetattr(fd , &options) != 0 )
		return (-1);

	if ( cfsetispeed(&options, speed) != 0 )
		return (-1);

	if ( cfsetospeed(&options, speed) != 0 )
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
