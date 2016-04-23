//==============================================================================
//
// Name        : COM-ports.h
// Author      : Fedor Dubrovin
// Description : Подпрограммы для работы с COM-портом
// Modified    : 2014-10-00 18-00
//
//==============================================================================

#ifndef COM_PORT_H_
#define COM_PORT_H_

#include <stdio.h>              // Standard Input / Output
#include <stdlib.h>             // General Utilities
#include <unistd.h>             // Symbolic Constants
#include <fcntl.h>              // File Control Operations
#include <termios.h>            // General Terminal Interface
#include <sys/signal.h>
#include <string.h>

//------------------------------------------------------------------------------

// Плохой файловый дескриптор
#define INVALID_FILE_DESCRIPTOR ( -1 )

//==============================================================================
//
// Работа с COM-портом (открытие, настройка, закрытие)
//
//==============================================================================
//------------------------------------------------------------------------------
// Открытие COM-порта и его настройка
//------------------------------------------------------------------------------
int open_com_port ( const char *COM_name, int baudrate );
//------------------------------------------------------------------------------
// Закрытие COM-порта
//------------------------------------------------------------------------------
bool close_com_port ();

// Отправление команд на com-port
void send_to_com(const char* buf, int count);

// Вывод данных с драйвера
int get_message ( char* buffer, int size);

//передача указателя на callback-функцию
void set_handler(void (*handler) (int));
#endif // COM_PORT_H_
