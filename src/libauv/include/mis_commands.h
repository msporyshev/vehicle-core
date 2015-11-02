/*
 * mis_commands.h
 *
 *  Created on: 17.03.2010
 *      Author: user
 */
//команды миссии

#ifndef MIS_COMMANDS_H_
#define MIS_COMMANDS_H_

#define OFF 0
#define ON 	1

//команды движения
extern int DIVING(float tack_time, float depth, float altitude);
extern int TACK(float tack_time, float heading, float velocity, int stab_mode, float altitude_depth);
extern int TACK_XY(float x, float y, float velocity, int stab_mode, float altitude_depth);
extern int TACK_P2P(float fromx, float fromy, float x, float y, float velocity, int stab_mode, float altitude_depth, float reach_error);
extern int DIRECT(float tack_time, float tx, float ty, float tz, float my, float mz);
//вспомогательные команды
extern void SET_ORIGIN_XY(double lon, double lat);
extern void COMPASS_CALIBRATION(int on_off);
//команды получения информации
extern float GET_HEADING(void);
extern float GET_PITCH(void);
//глубина (м), 0 поверхность, + вниз
extern float GET_DEPTH(void);
//высота над грунтом (м), + вверх
extern float GET_DD(void);
//комплексированные координаты (м)
extern float GET_X(void);
//комплексированные координаты
extern float GET_Y(void);
extern float GET_LONG(void);
extern float GET_LAT(void);
//скорость в горизонтальной плоскости (м/с)
extern float GET_V(void);
//команды управления устройствами
extern void SSS(int on_off);

//скрытые (не пользовательские) команды
extern void SIMULATION_MODE(void);
extern void REAL_MODE(void);
extern void END_MISSION(void);
extern char* rtodeg(float coord_rad);
extern float degtor(char *coord_deg);

#endif /* MIS_COMMANDS_H_ */
