#include "msg_miscom.h"
#include "utils/math_u.h"

//в этом файле написаны функции, которые заполняют структуру MSG_MISCOM_TYPE для команд телеуправления
//их использует и пульт и remote_test

//tack_time - в секундах, остальное - в процентах от -100 до 100
MSG_MISCOM_TYPE tu_direct(float tack_time, float tx, float ty, float tz, float my, float mz)
{
	MSG_MISCOM_TYPE res;
	res.command_code = DIRECT_;
	res.tx = tx;
	res.target_code = TARGET_TIME_;
	res.command_time = tack_time;
	res.ty = ty;
	res.tz = tz;
	res.my = my;
	res.mz = mz;
	return res;
}

//tack_time - в секнудах, heading - азимут в градусах, velocity - скорость в m/c, stab_mode - ASM или DSM, altitude_depth - м
MSG_MISCOM_TYPE tu_tack(float tack_time, float heading, float velocity, int stab_mode, float altitude_depth)
{
	MSG_MISCOM_TYPE res;
	res.command_code = TACK_;
	res.target_code = TARGET_TIME_;
	res.command_time = tack_time;
	while (heading < -180) heading += 360;
	while (heading > 180) heading -= 360;
	res.heading = heading * GR_to_R_;
	res.velocity = velocity;
	res.stab_mode = stab_mode;
	if (stab_mode == ASM)
		res.altitude = altitude_depth;
	else
		res.depth = altitude_depth;
	return res;
}

//x, y = m, velocity - скорость в m/c, stab_mode - ASM или DSM, altitude_depth - м
MSG_MISCOM_TYPE tu_tack_xy(float x, float y, float velocity, int stab_mode, float altitude_depth)
{
	MSG_MISCOM_TYPE res;
	res.command_code = TACK_XY_;
	res.target_code = TARGET_XY_;
	res.command_time = 0;
	res.x = x;
	res.y = y;
	res.velocity = velocity;
	res.stab_mode = stab_mode;
	if (stab_mode == ASM) 
        res.altitude = altitude_depth;
	else
        res.depth = altitude_depth;
	return res;
}

MSG_MISCOM_TYPE tu_tack_p2p(float x1, float y1, float x2, float y2, float velocity, int stab_mode, float altitude_depth)
{
	MSG_MISCOM_TYPE res;
	res.command_code = TACK_P2P_;
	res.target_code = TARGET_XY_;
	res.command_time = 0;
	res.x = x2;
	res.y = y2;
	res.fromx = x1;
	res.fromy = y1;
	res.velocity = velocity;
	res.stab_mode = stab_mode;
	if (stab_mode == ASM)
		res.altitude = altitude_depth;
	else 
		res.depth = altitude_depth;
	return res;
}
