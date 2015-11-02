//==============================================================================
// com_gbo.h 
//
// Описание пакетов сетевого взаимодействия и форматов данных для IPC
// (см. описание в пункте 4 докуметации).
//
//==============================================================================

#ifndef COM_GBO_H_
#define COM_GBO_H_

#include "ipc.h"

//Версия
#define GBO_COMMAND_VERSION ( 1 )

// Имя пакета
#define GBO_COMMAND_NAME "GBO_command"

// Структура пакета
typedef struct
{
  char Version;
  char A;                       // Channels (2 bits - left, right), Ftrans (1 bit), TVG (2 bits), Version(highest bit)
  char B;                       // PulseLength
  unsigned short C;             // DataLength
} GBO_command;

// Формат пакета
#define GBO_COMMAND_FORMAT "{char, char, char, ushort}"

#endif // COM_GBO_H_
