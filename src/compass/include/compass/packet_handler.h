/**
\file
\brief Реализация функций обмена данными с компасом

В данном файле находятся все фукции для обмена данными с компасом,
включая функции ожидания ответа от датчика с таймаутом и прочее.

\ingroup compass_node

*/
#pragma once

//Includes
#include <pthread.h>
#include <sys/timeb.h>

#include "compass/com_port.h"           // Подпрограммы для работы с COM-портом
#include "compass/usb_port.h"

#define BUFF_SIZE	500
#define TX_BUFF_SIZE 100
#define ACK_QUEUE_SIZE 10
#define TIMEOUT 1000

typedef struct {
  uint8_t  frame_control;
  uint8_t  length;
  uint8_t  id;
  uint8_t  payload[61];
} inemo_frame_struct;

#define QOS_SHIFT 0
#define QOS_MASK  3 << QOS_SHIFT
typedef enum {
  NORMAL_PRIORITY   = 0x00 << QOS_SHIFT,
  MEDIUM_PRIORITY   = 0x01 << QOS_SHIFT,
  HIGH_PRIORITY     = 0x02 << QOS_SHIFT,
  RFU_PRIORITY      = 0x03 << QOS_SHIFT
} QOS; // quality of service

#define FRAME_VERSION_SHIFT 2
#define FRAME_VERSION_MASK  3 << FRAME_VERSION_SHIFT
typedef enum {
  VER_1       = 0x00 << FRAME_VERSION_SHIFT,
  VER_DFU_1   = 0x01 << FRAME_VERSION_SHIFT,
  VER_DFU_2   = 0x02 << FRAME_VERSION_SHIFT,
  VER_DFU_3   = 0x03 << FRAME_VERSION_SHIFT
} FRAME_VERSION;

#define LMLF_SHIFT 4
#define LMLF_MASK  1 << LMLF_SHIFT
typedef enum {
  LM  = 0x00 << LMLF_SHIFT,
  LF  = 0x01 << LMLF_SHIFT
} LMLF; // quality of service

#define ASK_SHIFT 5
#define ASK_MASK  1 << ASK_SHIFT
typedef enum {
  NEED_ACK  = 0x01 << ASK_SHIFT,
  NO_ACK    = 0x00 << ASK_SHIFT
} ACK; // quality of service

#define FRAME_TYPE_SHIFT 6
#define FRAME_TYPE_MASK  3 << FRAME_TYPE_SHIFT
typedef enum {
  TYPE_CONTROL  = 0x00 << FRAME_TYPE_SHIFT,
  TYPE_DATA     = 0x01 << FRAME_TYPE_SHIFT,
  TYPE_ACK      = 0x02 << FRAME_TYPE_SHIFT,
  TYPE_NACK     = 0x03 << FRAME_TYPE_SHIFT
} FRAME_TYPE;

typedef enum {
  OUT_SAMPLE  = 0x00,
  OUT_TEMP    = 0x01,
  OUT_PRESS   = 0x02,
  OUT_MAG     = 0x04,
  OUT_GYRO    = 0x08,
  OUT_ACC     = 0x10,
  OUT_RAW     = 0x20,
  OUT_COMPASS = 0x40,
  OUT_AHRS    = 0x80
} OUTPUT_DATA;

#define RATE_SHIFT 3
typedef enum {
  RATE_1      = 0x00 << RATE_SHIFT,
  RATE_10     = 0x01 << RATE_SHIFT,
  RATE_25     = 0x02 << RATE_SHIFT,
  RATE_50     = 0x03 << RATE_SHIFT,
  RATE_30     = 0x04 << RATE_SHIFT,
  RATE_100    = 0x05 << RATE_SHIFT,
  RATE_400    = 0x06 << RATE_SHIFT,
  RATE_SYCH   = 0x07 << RATE_SHIFT
} OUTPUT_RATE;

#define INTARFACE_SHIFT 0
typedef enum {
    USB   = 0x00 << INTARFACE_SHIFT,
} OUTPUT_INTERFACE; // quality of service

#define ASK_DATA_SHIFT 6
typedef enum {
    ASK     = 0x01 << ASK_DATA_SHIFT,
    NO_ASK  = 0x00 << ASK_DATA_SHIFT
} OUTPUT_ASK; // quality of service

typedef enum {
  BUSY,
  OK
} STATUS;

typedef enum INEMO_M1_ID_e
{
  INEMO_M1_ID_CONNECT,
  INEMO_M1_ID_DISCONNECT,
  INEMO_M1_ID_RESET_BOARD,
  INEMO_M1_ID_ENTER_DFU_MODE,
  INEMO_M1_ID_TRACE       = 0x07,
  INEMO_M1_ID_LED_CONTROL,
  INEMO_GET_DEVICE_MODE,
  INEMO_GET_MCU_ID = 0x12,
  INEMO_GET_FW_VERSION,
  INEMO_GET_HW_VERSION,
  INEMO_IDENTIFY,
  INEMO_GET_AHRS_LIBRARY = 0x17,
  INEMO_GET_LIBRARIES,
  INEMO_GET_AVAILABLE_SENSORS,
  INEMO_SET_SENSOR_PARAMETER = 0x20,
  INEMO_GET_SENSOR_PARAMETER,
  INEMO_RESTORE_DEFAULT_PARAMETER,
  INEMO_SAVE_TO_FLASH = 0x23,
  INEMO_LOAD_FROM_FLASH = 0x24,
  INEMO_SET_OUTPUT_MODE = 0x50,
  INEMO_GET_OUTPUT_MODE,
  INEMO_START_ACQUISITION,
  INEMO_STOP_ACQUISITION,
  INEMO_GET_ACQ_DATA,
  INEMO_START_HIC_CALIBRATION = 0x60,
  INEMO_ABORT_HIC_CALIBRATION
}INEMO_M1_ID_t;

typedef enum ACK_STATUS_e
{
  EMPTY,
  WAIT_ANSWER,
  ANSWER_OK,
  ANSWER_ERROR
}ACK_STATUS_t;

typedef struct ACK_QUEUE_e
{
  uint8_t id;
  uint8_t status;
  uint8_t error;
}ACK_QUEUE_t;

typedef struct CALLBACK_e
{
  uint8_t id;
  uint8_t status;
  void (*callback)(inemo_frame_struct);
}CALLBACK_t;

/**
Инициализация обмена сообщениями с компасом
\param[in] msg Сообщение от компаса об ускорениях
*/
void packet_handler_init();

/**
Обработчик данных с USB/COM, который запущен вторым потоком
*/
void* data_handler(void* arg);

/**
Устанавливает уровень дебага в обработчике данных
\param[in] level уровень дебага
*/
void set_packet_debug_level(uint8_t level);

/**
Обработывает дпакеты приходящие с компаса в зависимости от
их ID
\param[in] frame пакет компаса
*/
void handle_data(inemo_frame_struct frame);

/**
Если этот пакет кем то ожидался, то функция сообщяет что данный
пакет пришел и запускает коллбек связанный с ID данного пакета функцией,
\param[in] ack_status статус пришедшего пакета
\param[in] frame пришедший пакет
*/
void handle_ack(inemo_frame_struct frame, int ack_status);

/**
\param[in]
*/
char insert_ack_queue(uint8_t id);

/**
\param[in]
*/
uint8_t wait_answer(uint8_t id, int timeout);

/**
\param[in]
*/
bool INEMO_send_command(inemo_frame_struct frame);

/**
\param[in]
*/
double get_fixate_time();

/**
\param[in]
*/
bool registrate_data_callback(int msg_id, void (*callback)(inemo_frame_struct));

/**
\param[in]
*/
bool deregistrate_data_callback(int msg_id);
