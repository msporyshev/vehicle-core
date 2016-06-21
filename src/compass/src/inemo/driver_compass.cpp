/**
\file
\brief Драйвер компаса

В данном файле находятся все фукции получения данных от компаса, их
интерпретации и подготовки физических величин.

\ingroup compass_node

*/

#include "compass/inemo/driver_compass.h"

#define CALIBRATION_COMPLETE    255

void handle_message(uint8_t* buffer, int len);
uint8_t* Reverse_array(uint8_t* data, uint8_t size);

uint8_t get_calibration_progress();
void parse_frame(inemo_frame_struct new_frame);

void data_callback(inemo_frame_struct frame);
void calibration_callback(inemo_frame_struct frame);

bool comport_status = 1;

uint8_t sensor_buffer[BUFF_SIZE];

pthread_t read_thread = 0;
bool data_busy = false;
int data_size;

uint8_t calibration_progress;
static uint8_t debug_level = 0;

sensor_struct_t sensors[9];

bool open_compass(const char *com_name, int baudrate)
{
    comport_status = usb_init_conection();
    //comport_status = open_com_port (com_name, baudrate);
    if(comport_status == 1)
        return 1;

    packet_handler_init();
    start_listen();

    sensors[SAMPLE_NUM].id      = OUT_SAMPLE;
    sensors[SAMPLE_NUM].size    = sizeof(uint16_t);
    sensors[ACCELEROMETER].id   = OUT_ACC;
    sensors[ACCELEROMETER].size = sizeof(Component_t);
    sensors[GYROSCOPE].id       = OUT_GYRO;
    sensors[GYROSCOPE].size     = sizeof(Component_t);
    sensors[MAGNETOMETER].id    = OUT_MAG;
    sensors[MAGNETOMETER].size  = sizeof(Component_t);
    sensors[PRESSURE].id        = OUT_PRESS;
    sensors[PRESSURE].size      = sizeof(uint32_t);
    sensors[TEMPERATURE].id     = OUT_TEMP;
    sensors[TEMPERATURE].size   = sizeof(uint16_t);
    sensors[ROTATION].id        = OUT_AHRS;
    sensors[ROTATION].size      = sizeof(Rotation_t);
    sensors[QUATERNION].id      = OUT_AHRS;
    sensors[QUATERNION].size    = sizeof(Quaternion_t);
    sensors[COMPASS].id         = OUT_COMPASS;
    sensors[COMPASS].size       = sizeof(Compass_t);

    registrate_data_callback(INEMO_START_ACQUISITION, data_callback);
    registrate_data_callback(INEMO_START_HIC_CALIBRATION, calibration_callback);

    return comport_status;
}

bool get_comport_status()
{
    return comport_status;
}

bool close_compass(void)
{
    if(debug_level) {
        printf("try to stop listen...\n");
    }
    stop_listen();

    if(debug_level) {
        printf("ok\n");
    }

    return (usb_close_connection());
    //return (close_com_port());
}

bool start_listen(void)
{
    if(read_thread != 0) {
        printf("Thread is already runnung\n");
        return 1;
    }

    printf("Request to create the thread... ");
    int err = pthread_create(&read_thread, NULL, &data_handler, NULL);
    if(err != 0) {
        printf("ERROR. can`t create thread, err: %s\n", strerror(err));
        return 1;
    }
    printf("Thread successful created. ID: %d\n", (uint)read_thread);
    return 0;
}

bool stop_listen(void)
{
    int err;

    if(read_thread == 0) {
        printf("Thread is not runnig\n");
        return 1;
    }

    printf("Request to cancel the thread... ");
    err = pthread_cancel(read_thread);
    if (err != 0) printf("ERROR. pthread_cancel, err: %s.\n", strerror(err));
    else          printf("Request successful.\n");

    printf("Waiting for thread stop... ");
    err = pthread_join(read_thread, NULL);
    if(!err)    printf("Waiting successful.\n");
    else        printf("ERROR. pthread_join, err: %s.\n", strerror(err));

    read_thread = 0;
    return 0;
}

void set_debug_level(uint8_t level)
{
    debug_level = level;
    set_packet_debug_level(level);
}

bool start_calibration(int timeout)
{
    bool status;

    status = INEMO_M1_Start_Calibration();
    if(debug_level)
        printf("INEMO_M1_Start_Calibration return %d\n", status);

    if(status)
        return 1;

    double start_time = get_timestamp();

    while(start_time + timeout > get_timestamp()) {
        if(get_calibration_progress() == CALIBRATION_COMPLETE) {
            return 0;
        } else {
            wait(1);
        }
    }

    INEMO_M1_Abort_Calibration();
    return 1;
}

uint8_t get_calibration_progress()
{
    return calibration_progress;
}

void data_callback(inemo_frame_struct frame)
{
    memcpy((void*)sensor_buffer, (void*)frame.payload, sizeof(char) * (frame.length - 1));

    for(uint8_t sens = 0; sens < SENSORS_NUM; sens++) {
        if(sensors[sens].sensor_status) {
            sensors[sens].data_status = 1;
        }
    }
}


void calibration_callback(inemo_frame_struct frame)
{
    static int iteration = 0;
    iteration++;
    calibration_progress = frame.payload[0];
    printf("iteration: %d\t. ", iteration);
    printf("Calibration progress: %d\r", (int)calibration_progress);
}

void set_compass_config(compass_config_struct conf)
{
    sensors[SAMPLE_NUM].sensor_status    = 1;
    sensors[ACCELEROMETER].sensor_status = 1;
    sensors[GYROSCOPE].sensor_status     = 1;
    sensors[MAGNETOMETER].sensor_status  = 1;
    sensors[PRESSURE].sensor_status      = 0;
    sensors[TEMPERATURE].sensor_status   = 0;
    sensors[ROTATION].sensor_status      = 1;
    sensors[QUATERNION].sensor_status    = 1;
    sensors[COMPASS].sensor_status       = 1;
}

bool INEMO_M1_Connect()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_M1_ID_CONNECT;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Disconnect()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_M1_ID_DISCONNECT;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Start_Acquisition()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_START_ACQUISITION;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Stop_Acquisition()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_STOP_ACQUISITION;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Config_Output()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 5;
    command_frame.id = INEMO_SET_OUTPUT_MODE;

    uint8_t sensors_set = 0;
    uint8_t shift = 0;

    memset(sensor_buffer, 0, BUFF_SIZE);

    for(int sens = 0; sens < SENSORS_NUM; sens++) {
        if(sensors[sens].sensor_status) {
            sensors[sens].pointer = (void*)(sensor_buffer + shift);
            shift += sensors[sens].size;
            sensors_set |= sensors[sens].id;
        }
    }

    command_frame.payload[0] = sensors_set;
    command_frame.payload[1] = RATE_400 | USB | NO_ASK;
    command_frame.payload[2] = 0;
    command_frame.payload[3] = 0;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Start_Calibration()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_START_HIC_CALIBRATION;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

bool INEMO_M1_Abort_Calibration()
{
    inemo_frame_struct command_frame;
    command_frame.frame_control = TYPE_CONTROL | VER_1 | NORMAL_PRIORITY | NEED_ACK | LM;
    command_frame.length = 1;
    command_frame.id = INEMO_ABORT_HIC_CALIBRATION;

    INEMO_send_command(command_frame);
    return wait_answer(command_frame.id, TIMEOUT);
}

uint8_t Get_Counter(uint16_t* data)
{
    if(!sensors[SAMPLE_NUM].sensor_status || !sensors[SAMPLE_NUM].data_status) {
        return 1;
    }

    uint16_t counter = *(uint16_t*)(sensors[SAMPLE_NUM].pointer);
    (*data) = *(uint16_t*)Reverse_array((uint8_t*)&counter, 2);
    sensors[SAMPLE_NUM].data_status = 0;
    return 0;
}

uint8_t Get_Accelerometer_Data(Component_t* data)
{
    if(!sensors[ACCELEROMETER].sensor_status || !sensors[ACCELEROMETER].data_status) {
        return 1;
    }

    Component_t accel = *(Component_t*)(sensors[ACCELEROMETER].pointer);
    (*data).X = *(short*)Reverse_array((uint8_t*)&accel.X, 2);
    (*data).Y = *(short*)Reverse_array((uint8_t*)&accel.Y, 2);
    (*data).Z = *(short*)Reverse_array((uint8_t*)&accel.Z, 2);
    sensors[ACCELEROMETER].data_status = 0;
    return 0;
}

uint8_t Get_Gyroscope_Data(Component_t* data)
{
    if(!sensors[GYROSCOPE].sensor_status || !sensors[GYROSCOPE].data_status) {
        return 1;
    }

    Component_t gyro = *(Component_t*)(sensors[GYROSCOPE].pointer);
    (*data).X = *(short*)Reverse_array((uint8_t*)&gyro.X, 2);
    (*data).Y = *(short*)Reverse_array((uint8_t*)&gyro.Y, 2);
    (*data).Z = *(short*)Reverse_array((uint8_t*)&gyro.Z, 2);
    sensors[GYROSCOPE].data_status = 0;
    return 0;
}

uint8_t Get_Magnetometer_Data(Component_t* data)
{
    if(!sensors[MAGNETOMETER].sensor_status || !sensors[MAGNETOMETER].data_status) {
        return 1;
    }

    Component_t magn = *(Component_t*)(sensors[MAGNETOMETER].pointer);
    (*data).X = *(short*)Reverse_array((uint8_t*)&magn.X, 2);
    (*data).Y = *(short*)Reverse_array((uint8_t*)&magn.Y, 2);
    (*data).Z = *(short*)Reverse_array((uint8_t*)&magn.Z, 2);
    sensors[MAGNETOMETER].data_status = 0;
    return 0;
}

uint8_t Get_Pressure_Data(uint32_t* data)
{
    if(!sensors[PRESSURE].sensor_status || !sensors[PRESSURE].data_status) {
        return 1;
    }

    uint32_t press = *(uint32_t*)(sensors[PRESSURE].pointer);
    (*data) = *(uint32_t*)Reverse_array((uint8_t*)&press, 4);
    sensors[PRESSURE].data_status = 0;
    return 0;
}

uint8_t Get_Temperature_Data(uint16_t* data)
{
    if(!sensors[TEMPERATURE].sensor_status || !sensors[TEMPERATURE].data_status) {
        return 1;
    }

    uint16_t temp = *(uint16_t*)(sensors[TEMPERATURE].pointer);
    (*data) = *(uint16_t*)Reverse_array((uint8_t*)&temp, 2);
    sensors[TEMPERATURE].data_status = 0;
    return 0;
}

uint8_t Get_Rotation_Data(Rotation_t* data)
{
    if(!sensors[ROTATION].sensor_status || !sensors[ROTATION].data_status) {
        return 1;
    }

    Rotation_t rotation = *(Rotation_t*)(sensors[ROTATION].pointer);
    (*data).Pitch = *(float*)Reverse_array((uint8_t*)&rotation.Pitch, 4);
    (*data).Roll    = *(float*)Reverse_array((uint8_t*)&rotation.Roll, 4);
    (*data).Yaw     = *(float*)Reverse_array((uint8_t*)&rotation.Yaw, 4);
    sensors[ROTATION].data_status = 0;
    return 0;
}

uint8_t Get_Quaternion_Data(Quaternion_t* data)
{
    if(!sensors[QUATERNION].sensor_status || !sensors[QUATERNION].data_status) {
        return 1;
    }

    Quaternion_t quaternion = *(Quaternion_t*)(sensors[QUATERNION].pointer);
    (*data).Q1 = *(float*)Reverse_array((uint8_t*)&quaternion.Q1, 4);
    (*data).Q2 = *(float*)Reverse_array((uint8_t*)&quaternion.Q2, 4);
    (*data).Q3 = *(float*)Reverse_array((uint8_t*)&quaternion.Q3, 4);
    (*data).Q4 = *(float*)Reverse_array((uint8_t*)&quaternion.Q4, 4);
    sensors[QUATERNION].data_status = 0;
    return 0;
}

uint8_t Get_Compass_Data(Compass_t* data)
{
    if(!sensors[COMPASS].sensor_status || !sensors[COMPASS].data_status) {
        return 1;
    }

    Compass_t compass = *(Compass_t*)(sensors[COMPASS].pointer);
    (*data).Heading   = *(float*)Reverse_array((uint8_t*)&compass.Heading, 4);
    (*data).Pitch     = *(float*)Reverse_array((uint8_t*)&compass.Pitch, 4);
    (*data).Roll      = *(float*)Reverse_array((uint8_t*)&compass.Roll, 4);
    sensors[COMPASS].data_status = 0;
    return 0;
}

uint8_t* Reverse_array(uint8_t* data, uint8_t size)
{
    for(uint8_t i = 0; i < size / 2; i++) {
        uint8_t temp = data[i];
        data[i] = data[size - i - 1];
        data[size - i - 1] = temp;
    }
    return data;
}

void wait(int delay_msec)
{
    int delay_usec = delay_msec * 1000;
    usleep (delay_usec);
}