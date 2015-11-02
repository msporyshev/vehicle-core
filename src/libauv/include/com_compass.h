//==============================================================================
//
// Name        : com_compass.h
// Author      : Fedor Dubrovin
// Description : Пакет "Команда управления компасом"
// Version     : 001
// Modified    : 2012-07-03 21-20
//
//==============================================================================

#ifndef COM_COMPASS_H_
#define COM_COMPASS_H_

//------------------------------------------------------------------------------
// Пакет "Команда управления компасом"
// Формируется диспетчером миссии

// Имя пакета
#define COM_COMPASS_NAME "COM_COMPASS"

// Версия формата пакета
#define COM_COMPASS_VERSION 001

// Тип, описывающий структуру пакета
typedef struct
{
  unsigned int version;         // Версия формата пакета
  double time;                  // Время формирования
  unsigned int command_code;    // Код команды
  float declination;            // Магнитное склонение (радианы)
} COM_COMPASS_TYPE, *COM_COMPASS_PTR;

// Описание формата пакета
#define COM_COMPASS_FORMAT "{uint, double, uint, float}"

//------------------------------------------------------------------------------

#endif /* COM_COMPASS_H_ */
