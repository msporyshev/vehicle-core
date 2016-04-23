#include "echometer/nmea_parser.h"

#define BUFFER_SIZE 2048

char nmea_rmc_parser( char *buf, void *data );

nmea_message_t nmea_messages[NAMES_SIZE];

nmea_depth_below_t nmea_depth_below_data;
nmea_depth_t nmea_depth_data;
nmea_temp_t nmea_temp_data;
nmea_date_t nmea_date_data;
nmea_orient_t nmea_orient_data;
bool new_data = false;
int nmea_depth_below_parser( char *buf, void *data );
int nmea_depth_parser( char *buf, void *data );
int nmea_temp_parser( char *buf, void *data );
int nmea_date_parser( char *buf, void *data );
int nmea_orient_parser( char *buf, void *data );

char* nmea_next_field( char* buf);
int nmea_next_sentence( char** buf, int len);
int nmea_check_end( char* buf, int len);

void nmea_field_init()
{
	nmea_messages[DEPTH_BELOW].id = "GPDBT";
	nmea_messages[DEPTH_BELOW].parser = nmea_depth_below_parser;
	nmea_messages[DEPTH_BELOW].data = &nmea_depth_below_data;

	nmea_messages[DEPTH].id = "GPDPT";
	nmea_messages[DEPTH].parser = nmea_depth_parser;
	nmea_messages[DEPTH].data = &nmea_depth_data;

	nmea_messages[TEMP].id = "GPMTW";
	nmea_messages[TEMP].parser = nmea_temp_parser;
	nmea_messages[TEMP].data = &nmea_temp_data;

	nmea_messages[DATE].id = "GPZDA";
	nmea_messages[DATE].parser = nmea_date_parser;
	nmea_messages[DATE].data = &nmea_date_data;

	nmea_messages[ORIENT].id = "YXXDR";
	nmea_messages[ORIENT].parser = nmea_orient_parser;
	nmea_messages[ORIENT].data = &nmea_orient_data;
}

char save_bufer[BUFFER_SIZE];
int actual_len = 0;
void nmea_parse( char* buf, int len)
{
	char* sentense_cur;
	memcpy(save_bufer + actual_len, buf, len);
	actual_len += len;

	// puts("Save buffer:-------");
	// for(int i = 0; i < actual_len; i++) {
		// printf("%c", save_bufer[i]);
	// }
	// puts("-------end");

	int cur_index;
	int next_index;
	while(1) {
		sentense_cur = save_bufer;
		// printf("Actual len: %d\n", actual_len);

		cur_index = nmea_next_sentence(&sentense_cur, actual_len);
		// puts("Next sentence:");
		// for(int i = 0; i < 100; i++) {
		// 	printf("%c", sentense_cur[i]);
		// }
		// puts("....end");

		//надо определить где сейчас конец и именно до него всё удалять
		next_index = nmea_check_end(sentense_cur, (actual_len - cur_index));
		if( next_index == (actual_len - cur_index) ) {
			// puts("End not found.");
			break;
		}
		// puts("End find.");
		// puts("Next sentence:");
		// for(int i = cur_index; i < (next_index + cur_index); i++) {
			// printf("%c", save_bufer[i]);
		// }
		// printf("\n");
		for(int name = 0; name < NAMES_SIZE; name++) {
			// printf("Try to find %s\n", nmea_messages[name].id);
			if( strncmp( sentense_cur, nmea_messages[name].id, 
				strlen( nmea_messages[name].id ) ) == 0 ) {
				new_data = true;
				nmea_messages[name].parser( sentense_cur, nmea_messages[name].data );
			}
		}

		for ( int j = (next_index + cur_index); j < actual_len; j++ ) {
			save_bufer[j - (next_index + cur_index)] = save_bufer[j];
		}
		actual_len -= (next_index + cur_index);

		// puts("Lost buffer:-------");
		// for(int i = 0; i < actual_len; i++) {
			// printf("%c", save_bufer[i]);
		// }
		// puts("-------end");
	}
}

bool is_data_new()
{
	if(new_data) {
		new_data = false;
		return true;
	} else {
		return false;
	}
}
int nmea_depth_below_parser( char *buf, void *data ){
	//$GPDBT,1.48,f,0.45,M,,*62
	nmea_depth_below_t* _data = (nmea_depth_below_t*)data;
	memset( _data, 0, sizeof( nmea_depth_below_t ) );

	buf = nmea_next_field( buf );
	_data->depth_feet = atof( buf );
	buf = nmea_next_field( buf );
	buf = nmea_next_field( buf );
	_data->depth_meters = atof( buf );
	// printf("depth feets: %f\n", _data->depth_feet);
	// printf("depth meters: %f\n", _data->depth_meters);
	return 0;
}
int nmea_depth_parser( char *buf, void *data ){
	//$GPDPT,0.5,0.0,100*4F
	nmea_depth_t* _data = (nmea_depth_t*)data;
	memset( _data, 0, sizeof( nmea_depth_t ) );

	buf = nmea_next_field( buf );
	_data->depth = atof( buf );
	buf = nmea_next_field( buf );
	_data->offset = atof( buf );
	buf = nmea_next_field( buf );
	_data->max_depth = atof( buf );

	// printf("depth: %f\n", _data->depth);
	// printf("offset: %f\n", _data->offset);
	// printf("max depth: %f\n", _data->max_depth);
	return 0;
}
int nmea_temp_parser( char *buf, void *data ){
	//$GPMTW,17.53,C*34
	nmea_temp_t* _data = (nmea_temp_t*)data;
	memset( _data, 0, sizeof( nmea_temp_t ) );

	buf = nmea_next_field( buf );
	_data->temperature = atof( buf );

	// printf("temperature: %f\n", _data->temperature);
	return 0;
}
int nmea_date_parser( char *buf, void *data ){
	//$GPZDA,014238.65,30,01,2012,09,540*52
	nmea_date_t* _data = (nmea_date_t*)data;
	memset( _data, 0, sizeof( nmea_date_t ) );

	_data->UTC = atof(buf);
	buf = nmea_next_field(buf);
	_data->day = atof(buf);
	buf = nmea_next_field(buf);
	_data->month = atof(buf);
	buf = nmea_next_field(buf);
	_data->year = atof(buf);
	buf = nmea_next_field(buf);
	_data->local_time_zone = atof(buf);
	return 0;
}
int nmea_orient_parser( char *buf, void *data ){
	return 0;
}

char* nmea_next_field( char* buf)
{
   while( *buf++ != ',' );
   return buf;
}

nmea_message_t* get_sentence(sentense_type name)
{
	return &(nmea_messages[name]);
}

int nmea_next_sentence( char** buf, int len)
{
	int cur_index = 0;
	while(cur_index != len) {
		if(*((*buf)++) == '$' ) {
			cur_index++;
			break;
		} else {
			cur_index++;
		}
	}
	return cur_index;
}

int nmea_check_end( char* buf, int len)
{
	int cur_index = 0;
	while(cur_index != len) {
		if(buf[cur_index] == '\n' || buf[cur_index] == '\r') 
			return cur_index;
		cur_index++;
	}
	return cur_index;
}