#ifndef SUPERVISOR_H
#define SUPERVISOR_H

//CANовские адреса ящиков супервизора
#define SUPREVISOR_INPUT_CAN_ADDR 101
#define SUPREVISOR_OUTPUT_CAN_ADDR 102
#define SUPREVISOR_OUTPUT_CAN_ADDR1 103

#ifdef ROBOSUB_AUV
//некоторые битики нам нужны
#define SUPERVISOR_DVL   (1 << 0)
#define SUPERVISOR_DEPTH (1 << 1)
#define SUPERVISOR_DSP   (1 << 8)
#define SUPERVISOR_FAN   (1 << 9)
#define SUPERVISOR_WS1   (1 << 12)
#define SUPERVISOR_WS2   (1 << 13)
#define SUPERVISOR_WS_TEST  (1 << 14)
#define SUPERVISOR_COMPENSATOR (1 << 14)
#define SUPERVISOR_MARKER1   (1 << 4)
#define SUPERVISOR_MARKER2   (1 << 5)
#define SUPERVISOR_TORPEDO1  (1 << 2)
#define SUPERVISOR_TORPEDO2  (1 << 3)
#define SUPERVISOR_COMPUTER  (1 << 11)

//порядок перечисления битиков важен - от него зависит открываются грабли или закрываются
#define SUPERVISOR_GRABBER1  (1 << 10)
#define SUPERVISOR_GRABBER2  (1 << 7)
#define SUPERVISOR_GRABBER1_OFFSET  10
#define SUPERVISOR_GRABBER2_OFFSET  7
#define GRABBER_DELAY 100
#define GRABBER_ITERATIONS 10

#endif

#ifdef MARK_AUV
#define SUPERVISOR_DVL (1 << 0)
#define SUPERVISOR_CTD (1 << 1)
#define SUPERVISOR_HANS (1 << 2)
#define SUPERVISOR_MAGNIT (1 << 3)
#define SUPERVISOR_COMPAS (1 << 4)
#define SUPERVISOR_SSS (1 << 5)
#define SUPERVISOR_24_1 (1 << 6)
#define SUPERVISOR_24_2 (1 << 7)
#define SUPERVISOR_FAN (1 << 8)
#define SUPERVISOR_HDD (1 << 9)
#define SUPERVISOR_LIGHTS (1 << 10)
#define SUPERVISOR_GPS_RADIO (1 << 11)
#define SUPERVISOR_ON_PC (1 << 12)
#define SUPERVISOR_RESET_PC (1 << 13)
#define SUPERVISOR_WS_TEST (1 << 14)

#define SUPERVISOR_WS1 (1 << 12)
#define SUPERVISOR_WS2 (1 << 13)
#define SUPERVISOR_COMPENSATOR1 (1 << 14)
#define SUPERVISOR_COMPENSATOR2 (1 << 15)
#endif

#define MSG_SUPERVISOR_BLINK_LIGHT_NAME "MSG_SUPERVISOR_BLINK_LIGHT"
#define MSG_SUPERVISOR_BLINK_LIGHT_FORMAT "{int, int, int}"
typedef struct {
	int period;			// период миганий в мс (не имеет смысла делать меньше чем LISTEN_PERIOD из .cpp)
	int light_time; 	// время когда маяк активен, мс (должен быть меньше периода, иначе будет равен периоду)
	int count; 			// число миганий ( 0 - будет мигать бесконечно)
} MSG_SUPERVISOR_BLINK_LIGHT_DATA;

#ifdef SURFACE_VEHICLE
#define SUPERVISOR_GPS_RADIO    (1 << 0)
#define SUPERVISOR_HANS         (1 << 1)
#define SUPERVISOR_THRUSTER_2   (1 << 6)
#define SUPERVISOR_THRUSTER_1   (1 << 7)
#define SUPERVISOR_COMPAS       (1 << 9)
#define SUPERVISOR_LIGHTS       (1 << 10)

#define SUPERVISOR_ON_PC        (1 << 12)
#define SUPERVISOR_RESET_PC     (1 << 13)
#define SUPERVISOR_WS_TEST      (1 << 14)

#define SUPERVISOR_WS2          (1 << 12)
#define SUPERVISOR_WS1          (1 << 13)
#define SUPERVISOR_COMPENSATOR1 (1 << 14)
#define SUPERVISOR_COMPENSATOR2 (1 << 15)
//байт 0 - напряжение питания компа * 10
//байт 1 - ток системы
//байт 2 - напряжение батареи * 10 (потом это может быть будет ток)
//байт 3 - температура 2
//байт 4 - температура 1
//байт 5 - общая вода * 10
#endif

#endif

