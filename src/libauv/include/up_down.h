#ifndef UP_DOWN_H
#define UP_DOWN_H

#include "msg_miscom.h" //сообщения типа MSG_MISCOM служат для телеуправления
#include "battery.h" //для структур с состоянием батарей

#define UP_DOWN_VERSION 8
#define THRUSTERS 4

/*Слово DOWNLOAD означает передачу информации вниз, на аппарат, UPLOAD - передача вверх*/

//БЛОК СООБЩЕНИЙ: СООБЩЕНИЯ ДЛЯ ПОДДЕРЖАНИЯ СВЯЗИ

//подтверждение о приёме сообщения сверху
#define MSG_UPLOAD_MESSAGE_CONFIRM_NAME "MSG_UPLOAD_MESSAGE_CONFIRM"
#define MSG_UPLOAD_MESSAGE_CONFIRM_FORMAT "{int, uint, double, string}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    char *message_name;
} MSG_UPLOAD_MESSAGE_CONFIRM;

//БЛОК СООБЩЕНИЙ: УПРАВЛЕНИЕ МИССИЕЙ

//передача миссии на аппарат
#define MSG_DOWNLOAD_MISSION_NAME "MSG_DOWNLOAD_MISSION"
#define MSG_DOWNLOAD_MISSION_FORMAT "{int, uint, double, {enum: 2}, int, <char: 5>, string}"
typedef enum{SRC_EXEC, SRC_TEXT, SRC_ONBOARD} MISSION_TYPE;
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    /*type = SRC_EXEC - загрузить исполняемый файл
    SRC_TEXT - загрузить текст программы
    SRC_ONBOARD - текущая миссия уже на борту*/
    MISSION_TYPE type;
    int length; //количество байт в s
    char *s; //исполняемый файл (type == SRC_EXEC), текст с кодом (type == SRC_TEXT) или идентификатор миссии (type == SRC_ONBOARD)
    char *name; //имя, под которым миссия будет сохранена на аппарате
} MSG_DOWNLOAD_MISSION;

//запуск миссии, подготовленной командой MSG_DOWNLOAD_MISSION
#define MSG_DOWNLOAD_RUN_NAME "MSG_DOWNLOAD_RUN"
#define MSG_DOWNLOAD_RUN_FORMAT "{int, uint, double}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
} MSG_DOWNLOAD_RUN;

//установка реального режима или режима моделирования
#define MSG_DOWNLOAD_MODE_NAME "MSG_DOWNLOAD_MODE"
#define MSG_DOWNLOAD_MODE_FORMAT "{int, uint, double, {enum: 1}, float}"
typedef enum{SIMULATION_MODE_MISSION, REAL_MODE_MISSION} MISSION_MODE;
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    MISSION_MODE mode;
    float time_acceleration; //во сколько раз ускорить время если включен режим симуляции
} MSG_DOWNLOAD_MODE;

//всегда отсылается в ответ на MSG_DOWNLOAD_MISSION
#define MSG_UPLOAD_TRANSFER_RESULT_NAME "MSG_UPLOAD_TRANSFER_RESULT"
#define MSG_UPLOAD_TRANSFER_RESULT_FORMAT "{int, uint, double, string}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    //текстовое сообщение с обратной связью в ответ на передачу миссии:
    //ошибки компиляции, ошибки несуществующих файлов или сообщение о том, что все ок.
    char *response;
} MSG_UPLOAD_TRANSFER_RESULT;

//запрос списка миссий, распложенных на аппарате
#define MSG_DOWNLOAD_REQUEST_MISSION_LIST_NAME "MSG_DOWNLOAD_REQUEST_MISSION_LIST"
#define MSG_DOWNLOAD_REQUEST_MISSION_LIST_FORMAT "{int, uint, double}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
} MSG_DOWNLOAD_REQUEST_MISSION_LIST;

//список названий миссий, распложенных на аппарате
#define MSG_UPLOAD_MISSION_LIST_NAME "MSG_UPLOAD_MISSION_LIST"
#define MSG_UPLOAD_MISSION_LIST_FORMAT "{int, uint, double, int, <string: 4>}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    int n;
    char **list;
} MSG_UPLOAD_MISSION_LIST;

//сообщение для запуска программ в терминале бортового компьютера
#define MSG_DOWNLOAD_TERMINAL_NAME "MSG_DOWNLOAD_TERMINAL"
#define MSG_DOWNLOAD_TERMINAL_FORMAT "{int, string, int}"
typedef struct {
    int callsign;
    char* command;
    int message_number;
} MSG_DOWNLOAD_TERMINAL;

//сообщение с ответом терминала
#define MSG_UPLOAD_TERMINAL_ANSWER_NAME "MSG_UPLOAD_ASWER_TERMINAL"
#define MSG_UPLOAD_TERMINAL_ANSWER_FORMAT "{int, string, int}"
typedef struct {
    int callsign;
    char* answer;
    int message_number;
} MSG_UPLOAD_TERMINAL_ANSWER;

//изменение коэффициентов регуляторов
#define MSG_DOWNLOAD_REGUL_PARAM_NAME "MSG_DOWNLOAD_REGUL_PARAM"
#define MSG_DOWNLOAD_REGUL_PARAM_FORMAT "{int, string, string, {enum: 1}, double, int}"
typedef enum{PARAM_TYPE_DOUBLE, PARAM_TYPE_INT} PARAM_TYPE;
typedef struct {
    int callsign;
    char *chapter;
    char *param;
    PARAM_TYPE ptype;
    double double_value;
    int int_value;
} MSG_DOWNLOAD_REGUL_PARAM;

//управление ходом выполнения миссии
#define MSG_DOWNLOAD_MISSION_CONTROL_NAME "MSG_DOWNLOAD_MISSION_CONTROL"
#define MSG_DOWNLOAD_MISSION_CONTROL_FORMAT "{int, uint, double, {enum: 4}}"
typedef enum{STOP_MISSION, SKIP_COMMAND_MISSION,
             FREEZE_MISSION, UNFREEZE_MISSION, REMOTE_CONTROL_MISSION} MISSION_COMMAND;
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    /*command = STOP_MISSION для полной отмены миссии
    FREEZE_MISSION для приостановки миссии
    UNFREEZE_MISSION для возобновления миссии
    SKIP_COMMAND_MISSION для снятия текущего задания
    REMOTE_CONTROL_MISSION для выполнения команды телеуправления, а какой?*/
    MISSION_COMMAND command;
} MSG_DOWNLOAD_MISSION_CONTROL;

//БЛОК СООБЩЕНИЙ: ПЕРЕДАЧА НАВЕРХ ИНФОРМАЦИИ О СОСТОЯНИИ АППАРАТА И СРЕДЫ

//информация о пространственном положении аппарата

#define MSG_UPLOAD_VEHICLE_SPACING_NAME "MSG_UPLOAD_VEHICLE_SPACING"
#define MSG_UPLOAD_VEHICLE_SPACING_FORMAT "{int, double, double, double, float, float, float, float, float, float, float, float, float, float, ubyte}"
typedef struct {
    int callsign;
    double time;
    double lat, lon; //широта и долгота (рад)
    float x, y; //координаты (м)
    float depth, height; //глубина (м), высота (м)
    float head, pitch, roll; //курс (рад от севера по часовой), дифферент (рад >0 - нос задран), крен (рад >0 - вправо)
    float velocity; //абсолютная величина скорости (м/с)
    float vel_n; //скорость проекция на север, (м/с)
    float vel_e; //скорость проекция на восток, (м/с)
    unsigned char mode; //SIMULATION_MODE_MISSION или REAL_MODE_MISSION (формат описан выше в MSG_DOWNLOAD_MODE)
} MSG_UPLOAD_VEHICLE_SPACING;

//информация c полезных датчиков
#define MSG_UPLOAD_SENSORS_NAME "MSG_UPLOAD_SENSORS"
#define MSG_UPLOAD_SENSORS_FORMAT "{int, double, double, double}"
typedef struct {
    int callsign;
    double time;
    double t; //температура (град. C)
    double solt; //соленость (г/м^3)
} MSG_UPLOAD_SENSORS;

//информация о состоянии СПУ
/*это сообщение должно отправляться каждый раз при смене состояния СПУ
  и может выступать в роли подтверждения, когда пользователь как-то влияет на ход миссии,
  Если использовать elapsed_time и time_left, то можно отправлять пакет периодически, но редко
*/
/*vehicle_state = STATE_NONE - аппарат ничего не делает
STATE_MISSION - выполняется миссия
STATE_REMOTE - выполняется команда телеуправления
STATE_FROZEN - миссия приостановлена
STATE_ALARM - работы остановлены, включен аварийный режим*/
typedef enum{STATE_NONE, STATE_MISSION, STATE_REMOTE, STATE_FROZEN, STATE_ALARM} VEHICLE_STATE;
#define MSG_UPLOAD_MISSION_STATE_NAME "MSG_UPLOAD_MISSION_STATE"
#define MSG_UPLOAD_MISSION_STATE_FORMAT "{int, byte, string, float, float}"
typedef struct {
    int callsign;
    unsigned char vehicle_state;
    char* current_function; //нуль-терминальная строка с текущей фукнцией и ее параметрами
    //по показателям времени можно следить, когда изменилась текущая функция
    float elapsed_time; //время, прошедшее с начала выполнения текущей функции (сек)
    float time_left; //оценка количества оставшегося времени для завершения текущей функции (сек)
} MSG_UPLOAD_MISSION_STATE;

//информация о состоянии КАС
//CAS_EL_NO - всё хорошо
//CAS_EL_LOW - что-то случилось, но можно продолжать работу
//CAS_EL_HI - все плохо
typedef enum {CAS_EL_NO, CAS_EL_LOW, CAS_EL_HI} CAS_ERROR_LEVEL;
#define MSG_UPLOAD_ALARM_NAME "MSG_UPLOAD_ALARM"
//#define MSG_UPLOAD_ALARM_FORMAT "{uint, double, {enum: 2}, string}"
#define MSG_UPLOAD_ALARM_FORMAT "{int, uint, double, {enum: 2}}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    CAS_ERROR_LEVEL err_level;
    //char *message; //нуль-терминальная строка с подробным сообщением об ошибке
} MSG_UPLOAD_ALARM;

//БЛОК СООБЩЕНИЙ: ОБЩЕНИЕ С СУПЕРВИЗОРОМ И БАТАРЕЕЙ

//команда на включение/выключение питания
#define MSG_DOWNLOAD_SUPERVISOR_POWER_NAME "MSG_DOWNLOAD_SUPERVISOR_POWER"
#define MSG_DOWNLOAD_SUPERVISOR_POWER_FORMAT "{int, int, int}"
typedef struct {
    int callsign;
    int power_on_off;
    int power_offset;
} MSG_DOWNLOAD_SUPERVISOR_POWER;

//информация от супервизора
#ifdef MARK_AUV

#define MSG_UPLOAD_SUPERVISOR_DATA_NAME "MSG_UPLOAD_SUPERVISOR_DATA"
#define MSG_UPLOAD_SUPERVISOR_DATA_FORMAT "{int, int, float, float, int, float, float, float, float, int}"
typedef struct {
    int callsign;
    int flags; //флаги показывают факт потребления тока устройствами + компенсаторы + датчики воды
    float pc_volts; //напряжение на компьютере
    float current; //суммарный ток
    int csw; //датчик вода/суша. 1 - вода
    float csw_volts; //напряжение на датчике суша/вода
    float temperature1, temperature2; //датчики температуры
    float ws_ext; //напряжение на общем датчике затекания
    int is_underwater; //0 - мы на суше, 1 - мы в воде (на основе csw)
} MSG_UPLOAD_SUPERVISOR_DATA;

#elif defined ROBOSUB_AUV

#define MSG_UPLOAD_SUPERVISOR_DATA_NAME "MSG_UPLOAD_SUPERVISOR_DATA"
#define MSG_UPLOAD_SUPERVISOR_DATA_FORMAT "{int, float, float, float, float, float, int, float, float, float, int}"
typedef struct {
    int flags; //флаги показывают факт потребления тока устройствами + компенсатор + датчики воды + батарея
    float pc_volts; //напряжение на компьютере
    float total_volts; //напряжение от батареи
    float ws_ext; //напряжение на общем датчике затекания
    float current; //ток системы
    float total_current; //общий ток
    int csw; //датчик вода/суша. 1 - вода
    float csw_volts; //напряжение на датчике суша/вода
    float temperature; //температура автопилота
    float depth; //глубина в этом аппарате заведена на супервизор
    int is_underwater; //0 - мы на суше, 1 - мы в воде (на основе csw и показаний датчика глубины)
} MSG_UPLOAD_SUPERVISOR_DATA;

#elif defined(SURFACE_VEHICLE) || defined(PULT)

#define MSG_UPLOAD_SUPERVISOR_DATA_NAME "MSG_UPLOAD_SUPERVISOR_DATA"
#define MSG_UPLOAD_SUPERVISOR_DATA_FORMAT "{int, int, float, float, int, float, float, float, float, int, float, float}"
typedef struct {
    int callsign;
    int flags; //флаги показывают факт потребления тока устройствами + компенсаторы + датчики воды
    float pc_volts; //напряжение на компьютере
    float current; //суммарный ток
    int csw; //датчик вода/суша. 1 - вода
    float csw_volts; //напряжение на датчике суша/вода
    float temperature1; //датчики температуры
    float temperature2; //датчики температуры
    float ws_ext; //напряжение на общем датчике затекания
    int is_underwater; //0 - мы на суше, 1 - мы в воде (на основе csw)
    float voltage;
    float total_current;
} MSG_UPLOAD_SUPERVISOR_DATA;

#else
#error unknown vehicle
#endif

//команда на переход в режим заряда или рабочий режим
#define MSG_DOWNLOAD_BATTERY_COMMAND_NAME "MSG_DOWNLOAD_BATTERY_COMMAND"
#define MSG_DOWNLOAD_BATTERY_COMMAND_FORMAT "{int, uint, double, {enum: 10}, int, int}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    BATTERY_COMMAND command; //см. battery.h для списка возможных команд
    int battery_number; // 1, 2, 3 команда "включить/выключить ключ" требует номер батареи. Eсли команде все равно, то поле игнорируется
    int on_off; //0, 1 - при управлении ключами установить, иначе игнорируется
} MSG_DOWNLOAD_BATTERY_COMMAND;

//информация от батарей
#define MSG_UPLOAD_BATTERY_STATE_NAME "MSG_UPLOAD_BATTERY_STATE"
#define MSG_UPLOAD_BATTERY_STATE_FORMAT "{int, uint, double, " \
    "float, " \
    "[ubyte: 6], " \
    "{float, float, float}, " \
    "{float, float, float}, " \
    "{ubyte, ubyte, ubyte, ubyte, ubyte, ubyte}, " \
    "{float, float}, " \
    "{float, float}, " \
    "{float, float}, " \
    "{float, float, float}" \
    "}"
typedef struct {
    int callsign;
    unsigned int version;
    double time;
    float fraction; //доля заряда - число от 0 до 1
    unsigned char state_bytes[6];
    work_data1 wd1;
    work_data2 wd2;
    work_data3 wd3;
    charge_data1 cd1;
    charge_data2 cd2;
    charge_data3 cd3;
    charge_data4 cd4;
} MSG_UPLOAD_BATTERY_STATE;

#endif // UP_DOWN_H

