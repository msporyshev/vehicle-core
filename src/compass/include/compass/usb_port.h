/**
\file
\brief Обьявление функций работы с USB-устройством

В данном файле находятся все фукции для работы с USB-устройством:
инициализация, чтение, запись.

\ingroup compass_node

*/
#pragma once

#include <stdio.h>              // Standard Input / Output
#include <stdlib.h>             // General Utilities
#include <unistd.h>             // Symbolic Constants
#include <fcntl.h>              // File Control Operations
#include <termios.h>            // General Terminal Interface
#include <string.h>

#include <libusb-1.0/libusb.h>



bool usb_init_conection();

bool usb_send_data(uint8_t* data, int size);

int usb_read_data(uint8_t* data);

bool usb_close_connection();