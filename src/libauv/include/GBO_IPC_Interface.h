/*GBO_IPC_Interface.h – описание пакетов сетевого взаимодействия и форматов данных для IPC (описание в пункте 4).
 *
 * GBO_IPC_Interface.h
 *
 *  Created on: 05.08.2010
 *      Author: Varlamov D.A.
 */

#ifndef GBO_IPC_INTERFACE_H_
#define GBO_IPC_INTERFACE_H_

//Версия
#define GBO_Interface_Version 01
//Входящий в драйвер пакет
typedef struct
{
    int callsign;
	char Version;
	char A;				// Channels (2 bits - left, right), Ftrans (1 bit), TVG (2 bits), Version(highest bit)
	char B;				// PulseLength
	unsigned short C;		// DataLength
} GBO_command;

//Формат пакета (для IPC)
#define GBO_command_format "{int, char, char, char, ushort}"
#define GBO_command_name "GBO_command"

FORMATTER_PTR GBO_data_formatter;

#endif /* GBO_IPC_INTERFACE_H_ */

