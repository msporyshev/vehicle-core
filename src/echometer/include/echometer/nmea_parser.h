#ifndef NMEA_PRSER_H
#define NMEA_PRSER_H

#include <stdlib.h>             // General Utilities
#include <string.h>
#include <stdio.h>

typedef enum SENTENSE_NAMES
{
	DEPTH_BELOW, // Depth Below Transducer (value expressed in feet, meters and fathoms)
	DEPTH, // Depth (depth in meters, offset, max. depth in meters)
	TEMP, // Mean Temperature of Water (Water temperature in degrees centigrade)
	DATE, // Date & Time (UTC, day, month, year, and local time zone)
	ORIENT,  // Roll and pitch
	NAMES_SIZE
} sentense_type;

typedef struct _nmea_message {
  const char *id; //идентификатор сообщения
  int (*parser)( char *buf, void *data ); //парсер сообщения
  void *data; //результат парсинга сообщения
} nmea_message_t;


typedef struct nmea_depth_below {
	double depth_feet;
	double depth_meters;
}nmea_depth_below_t;

typedef struct nmea_depth {
	double depth;
	double offset;
	double max_depth;
}nmea_depth_t;

typedef struct nmea_temp {
	double temperature;
}nmea_temp_t;

typedef struct nmea_date {
	int UTC;
	int day;
	int month;
	int year;
	int local_time_zone;
}nmea_date_t;

typedef struct nmea_orient {
	double pitch;
	double roll;
}nmea_orient_t;


void nmea_field_init();

void nmea_parse( char* buf, int len);

nmea_message_t* get_sentence(sentense_type name);

bool is_data_new();
#endif /*NMEA_PRSER_H*/