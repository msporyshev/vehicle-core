#include "echometer/driver_echometer.h"

// Дефайны
#define THREAD_NUM  123
// Локальные функции
void handle_altimeter_sample(char* buf, int len);

void handle_echosounder(char* buf, int len);

void handle_altimeter_nmea(char* buf, int len);

void* data_handler(void* arg);

double get_fixate_time();
// Глобальные переменные
Workmode current_workmode = ALTIMETER_SIMPLE;
echosouder_config_struct current_config;
pthread_t read_thread = 0;
echosouder_data_struct echosouder_data; 
bool data_busy = false;

bool open_echosounder(const char *COM_name, int baudrate)
{
    bool status = 1;
    status = open_com_port (COM_name, baudrate);

    if ( status != 0 ) {
        puts ( "ERROR: cannot open com port" );
        puts ( "Check name and baudrate of COM-port" );
    } else {
        nmea_field_init();
    }
    return status;
}

bool close_echosounder (void)
{
    echosounder_stop();
    stop_listen();
    bool status = close_com_port();
    return (status);
}

bool start_listen (void)
{
    if(read_thread != 0) {
        printf("Thread is already runnung\n");
        return 1;
    }

    printf("Request to create the thread... ");
    int err = pthread_create(&read_thread, NULL, &data_handler, NULL);
    if(err != 0) {
        printf("ERROR. can`t create thread, err: %s\n", strerror(err));
        read_thread = 0;
        return 1;
    }
    printf("Thread successful created. ID: %d\n", (uint)read_thread);
    return 0;
}

bool stop_listen (void)
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
    if(!err)   printf("Waiting successful.\n");
    else       printf("ERROR. pthread_join, err: %s.\n", strerror(err));

    read_thread = 0;
    return 0;
}

void print_program_header ( void )
{
    puts("");
    puts("---------------------------------------------");
    puts("         Driver for echometer ECS400         ");
    puts("                                             ");
    puts("                 2015-06-20                  ");
    puts("---------------------------------------------");
    puts("");
}

#define BUFF_SIZE ( 2048 )

void* data_handler(void* arg)
{
    int len = 0;
    char buff[BUFF_SIZE];
    
    //сбрасываем накопленное в лог
    puts("ECHOSOUNDER FEEDBACK START\n");
    for(int i = 0; i < 3; i++) {
        wait(1000);
        len += get_message(buff, BUFF_SIZE);
        printf("Get %d bytes from echometer\n", len);
        //for ( int j = 0; j < len; j++ )
        //  printf ( "%c", buff[j] );    
    }
    puts("ECHOSOUNDER FEEDBACK END\n");

    puts("Start reading...\n");
    while(true)
    {
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        len = get_message(buff, BUFF_SIZE);
        if(len > 0) {
                // for ( int j = 0; j < len; j++ )
                    // printf ( "%c", buff[j] );
            switch(current_workmode) {
                case ALTIMETER_SIMPLE:
                    handle_altimeter_sample(buff, len);
                    break;
                case ECHOSOUNDER:
                    handle_echosounder(buff, len);
                    break;
                case ALTIMETER_NMEA:
                    handle_altimeter_nmea(buff, len);
                    break;
                case ECHOSOUNDER_200:
                case ECHOSOUNDER_500:
                case ECHOSOUNDER_1000:
                    handle_echosounder(buff, len);
                    break;

            }
        }
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_testcancel(); //cancel point
        wait(50);
    }
}


char save_buf_sample[200];
int cur_pos = 0;
void handle_altimeter_sample(char* buf, int len)
{
    memcpy(save_buf_sample + cur_pos, buf, len);
    cur_pos += len;

    int i = 1;
    while ( i < cur_pos ) {
        if ( (save_buf_sample[i] == 0x0A) && (save_buf_sample[i - 1] == 0x0D) ) {
            
            if(i > 2) {
                double val = atof (save_buf_sample);
                //printf ( "Read altitude: %f\n", val );
                echosouder_data.depth_below = val;
                echosouder_data.depth = echosouder_data.depth_below + current_config.offset / 1000;

                if(echosouder_data.depth <= current_config.deadzone / 1000.0)
                    echosouder_data.bad_data = 1;

                echosouder_data.new_data = true;
                echosouder_data.time = get_fixate_time();
            }

            for ( int j = i + 1; j < cur_pos; j++ ) {
                save_buf_sample[j - i - 1] = save_buf_sample[j];
            }

            cur_pos -= i + 1;
            i = 0;
        }
        i++;
    }
}

void handle_echosounder(char* buf, int len)
{

}


void handle_altimeter_nmea(char* buf, int len)
{
    nmea_parse(buf, len);

    nmea_message_t* depth_below_mes;
    depth_below_mes = get_sentence(DEPTH_BELOW);
    nmea_depth_below_t* depth_below_data = (nmea_depth_below_t*)depth_below_mes->data;
    // cout << "DEPTH_BELOW" << endl;
    // cout << "depth_feet: " << depth_below_data->depth_feet << endl;
    // cout << "depth_meters: " << depth_below_data->depth_meters << endl;

    nmea_message_t* depth_mes;
    depth_mes = get_sentence(DEPTH);
    nmea_depth_t* depth_data = (nmea_depth_t*)depth_mes->data;
    // cout << "DEPTH" << endl;
    // cout << "depth: " << depth_data->depth << endl;
    // cout << "offset: " << depth_data->offset << endl;
    // cout << "max_depth: " << depth_data->max_depth << endl;


    nmea_message_t* temp_mes;
    temp_mes = get_sentence(TEMP);
    nmea_temp_t* temp_data = (nmea_temp_t*)temp_mes->data;
    // cout << "TEMP" << endl;
    // cout << "temperature: " << temp_data->temperature << endl;

    data_busy = true;

    echosouder_data.depth = depth_below_data->depth_meters;
    echosouder_data.depth_below = echosouder_data.depth - current_config.offset / 1000;

    if(echosouder_data.depth <= current_config.deadzone / 1000.0) {
        echosouder_data.bad_data = 1;
    } else {
        echosouder_data.bad_data = 0;
    }
    
    // Несмотря на кажущуюся абсурдность данной композиции всё верно.
    // В текущей версии прошивки датчика depth_below_data и depth
    // это одно и тоже, отличается только точностью величин.
    // Поэтому приходится изворачиваться что бы получить отдельно
    // значения глубины и высоты над дном.
    // Скажем привет корейским компаниям.

    echosouder_data.temperature = temp_data->temperature;
    echosouder_data.new_data = is_data_new();
    echosouder_data.time = get_fixate_time();
    data_busy = false;
}

bool get_data(echosouder_data_struct& data)
{
    while(data_busy == true);

    data = echosouder_data;
    
    if(echosouder_data.new_data) {
        echosouder_data.new_data = !(echosouder_data.new_data); 
    }
    return data.new_data;
}

bool get_config(echosouder_config_struct& config)
{
    config = current_config;
    return 0;
}

void echosounder_start()
{
    int len;
    char message[30];
    char comand[] = "#go";
    len = sprintf(message, "%s", comand);
    send_to_com(message, len);
    wait(DEVICE_WAIT);
}

void echosounder_stop()
{
    int len;
    char message[30];
    char comand[] = "#";
    len = sprintf(message, "%s", comand);
    send_to_com(message, len);
    wait(DEVICE_WAIT);
}

void echosounder_reset()
{
    echosounder_stop();
    echosounder_start();
}

void echosounder_set_config(echosouder_config_struct config)
{
    stop_listen();
    echosounder_stop();

    set_workmode(config.workm);
    set_range(config.range);
    set_interval(config.interval);
    set_threshold(config.threshold);
    set_offset(config.offset);
    set_deadzone(config.deadzone);
    set_syncmode(config.syncextern);
    set_syncedge(config.syncextmod);
    
    echosounder_start();
    start_listen();
    current_config = config;
}

void set_workmode(Workmode workm)
{
    char comand[] = "#output";
    char val[10];
    sprintf(val, "%d", (int)workm);
    send_command(comand, val);
    current_workmode = workm;
    switch(workm) {
        case (ALTIMETER_SIMPLE):
            echosouder_data.temperature = -999;
        break;
        default:
            echosouder_data.depth_below = -999;
            echosouder_data.depth = -999;
            echosouder_data.depth_offset = -999;
            echosouder_data.depth_max = -999;
            echosouder_data.temperature = -999;
    }
}

void set_range(int range_mm)
{
    if(range_mm > 100000 || range_mm < 1000) {
        puts("ERROR: range_mm out of range [1000; 100000].");
        return;
    }
    char comand[] = "#range";
    char val[10];
    sprintf(val, "%d", (int)range_mm);
    send_command(comand, val);
    echosouder_data.depth_max = (double )range_mm / 1000;
}

void set_interval(double interval_sec)
{
    if(interval_sec >= 3600 || interval_sec < 0.1) {
        puts("ERROR: interval_sec out of range [0.1; 3600].");
        return;
    }
    char comand[] = "#interval";
    char val[10];
    sprintf(val, "%.1f", (double)interval_sec);
    send_command(comand, val);
}

void set_threshold(int level)
{  
    if(level >= 10000 || level < 0) {
        puts("ERROR: level out of range [0; 10000].");
        return;
    }
    char comand[] = "#threshold";
    char val[10];
    sprintf(val, "%d", level);
    send_command(comand, val);
}

void set_offset(int offset_mm)
{
    char comand[] = "#offset";
    char val[10];
    sprintf(val, "%d", (int)offset_mm);
    send_command(comand, val);
    echosouder_data.depth_offset = (double )offset_mm / 1000;
}

void set_deadzone(int level_mm)
{
    char comand[] = "#deadzone";
    char val[10];
    sprintf(val, "%d", (int)level_mm);
    send_command(comand, val);  
}

void set_txlength(int length_mcs)
{
    char comand[] = "#txlength";
    char val[10];
    sprintf(val, "%d", (int)length_mcs);
    send_command(comand, val); 
}

void set_syncmode(Syncmode mode)
{
    char comand[] = "#syncextern";
    char val[10];
    sprintf(val, "%d", (int)mode);
    send_command(comand, val);
}

//Установить синхронизацию по возврастающей или спадающей волне
void set_syncedge(Syncedge mode)
{
    char comand[] = "#syncextmod";
    char val[10];
    sprintf(val, "%d", (int)mode);
    send_command(comand, val);  
}

void send_command(char* comand, char* val)
{
    int len;
    char message[30];
    len = sprintf(message, "%s %s", comand, val);

    send_to_com(message, len);
    wait(DEVICE_WAIT);
}

void wait(int delay_msec)
{
    int delay_usec = delay_msec * 1000;
    usleep (delay_usec);
}

// Есть в unils. Здесь находится специально для переносимости
// Возвращает текущее время c миллисекундной точностью в формате double
double get_fixate_time()
{
    struct timeb timebuf;
    ftime(&timebuf);
    return ( (double)timebuf.time + (double)timebuf.millitm / 1000 );
}