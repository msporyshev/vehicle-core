/*
 * NMEA_decode.h
 *
 *  Created on: 15.09.2010
 *      Author: user
 */

#ifndef NMEA_DECODE_H_
#define NMEA_DECODE_H_

// Расчет контрольной суммы для NMEA 0183
int Calc_NMEA_CS(char* c);

// Выделение одного поля, начиная с позиции start
int GetField(char* c, int* start, char** field);

// Выделение целого числа, начиная с позиции start
int GetInt(char* c, int* start, int* value);

// Выделение целого числа в шестнадцатеричном виде, начиная с позиции start
int GetHex(char* c, int* start, int* value);

// Выделение double, начиная с позиции start
int GetDouble(char* c, int* start, double* value);

// Выделение символа, начиная с позиции start
int GetChar(char* c, int* start, char* value);

#endif /* NMEA_DECODE_H_ */
