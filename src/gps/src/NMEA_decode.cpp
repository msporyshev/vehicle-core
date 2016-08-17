/*********************************************************************
 *
 * Name        : NMEA_decode.h
 * Author      : Fedor Dubrovin
 * Description : Обработка строк в формате NMEA 0183
 * Version     : 001
 * Modified    : 2010_12_16 13-00
 *
 ********************************************************************/

#include <stdio.h>				// Input/output
#include <stdlib.h>				// General utilities
#include <unistd.h>				// Symbolic Constants
#include <string.h>				// String handling
#include <fcntl.h>				// File Control Operations
#include <termios.h>			// General Terminal Interface

//------------------------------------------------------------------------------
// Расчет контрольной суммы для NMEA 0183
int Calc_NMEA_CS(char* c)
{
	int len, i;
	char XOR;

	if (c[0] != '$')
		return (-1);									// Нет символа '$' в начале строки

	len = strlen(c);

	if ( (len < 1) || (len > 120) )
		return (-2);									// Недопустимая длина строки

	XOR = 0x00;
	i = 1;

	while ((i < len) && (c[i] != '*'))
		XOR ^= c[i++];

	if (i == len)
		return (-3);									// Не найден символ '*'

	return (XOR);
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Выделение одного поля, начиная с позиции start
int GetField(char* c, int* start, char** field)
{
	if (*start > strlen(c))
	{
		return (-2);									// Строка закончилась
	}

	int len;

	c = (char*)(*((int*)(&c)) + *start);

	len = strcspn(c, ",*");

	*start += len + 1;

	if (len <= 0)
	{
		return (-1);									// Пустое поле
	}

	char* block = (char*)malloc(len + 1);

	strncpy(block, c, len);
	block[len] = '\0';

	*field = block;

	return (0);
}

// Выделение целого числа, начиная с позиции start
int GetInt(char* c, int* start, int* value)
{
	char* field;

	if ( GetField(c, start, &field) != 0 )
	{
		*value = 0;

		return (-1);
	}

	if ( sscanf(field, "%d", value) != 1)
	{
		*value = 0;
	}

	free(field);
	return (0);
}

// Выделение целого числа в шестнадцатеричном виде, начиная с позиции start
int GetHex(char* c, int* start, int* value)
{
	char* field;

	if ( GetField(c, start, &field) != 0 )
	{
		*value = 0;

		return (-1);
	}

	if ( sscanf(field, "%x", value) != 1)
	{
		*value = 0x00;
	}

	free(field);
	return (0);
}

// Выделение double, начиная с позиции start
int GetDouble(char* c, int* start, double* value)
{
	char* field;

	*value = 0.0;

	if ( GetField(c, start, &field) != 0 )
	{
		*value = 0.0;

		return (-1);
	}

	if ( sscanf(field, "%lf", value) != 1)
	{
		*value = 0.0;

		free(field);
		return (-1);
	}

	free(field);
	return (0);
}

// Выделение символа, начиная с позиции start
int GetChar(char* c, int* start, char* value)
{
	char* field;

	if ( GetField(c, start, &field) != 0 )
	{
		*value = ' ';

		return (-1);
	}

	if ( sscanf(field, "%c", value) != 1)
	{
		*value = ' ';
	}

	free(field);
	return (0);
}
