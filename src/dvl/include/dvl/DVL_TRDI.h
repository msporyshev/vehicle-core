#ifndef DVL_TRDI_H
#define DVL_TRDI_H

#include <string>

#define READ_BUF_SUZE 256

enum reference_type {
    REF_TYPE_WATERMASS,
    REF_TYPE_BOTTOMTRACK
};

struct orientation {
    float pitch;
    float roll;
    float heading;
    bool new_data;
};

struct date_scalling {
    long int data;
    float salinity;
    float temperature;
    float depth;
    float soundspeed;
    int selftest_result;
    bool new_data;
};

struct instrument_reference_vel {
    int forward;
    int right;
    int down;
    int error;
    char status;
    bool new_data;
};

typedef struct instrument_reference_vel inst_vel_watermass;
typedef struct instrument_reference_vel inst_vel_bottomtrack;

struct ship_reference_vel {
    int transverse;
    int longitudinal;
    int normal;
    int status;
    bool new_data;
};

typedef struct ship_reference_vel ship_vel_watermass;
typedef struct ship_reference_vel ship_vel_bottomtrack;

struct earth_reference_vel {
    int east;
    int north;
    int upward;
    int status;
    bool new_data;
};

typedef struct earth_reference_vel earth_vel_watermass;
typedef struct earth_reference_vel earth_vel_bottomtrack;

struct earth_reference_dist {
    float east;
    float north;
    float upward;
    float range;
    float time;
    bool new_data;
};

typedef struct earth_reference_dist earth_dist_watermass;
typedef struct earth_reference_dist earth_dist_bottomtrack;

class DvlTrdiDriver
{
    char* get_timestamp();

    //чтение и расшифровка данных пакета
    bool decoding_data(char* list);

    // Получение данных с устройства
    bool dvl_read();
public:
    DvlTrdiDriver();
    bool dvl_start(const char* com_port, int baudrate);
    bool dvl_stop();
    bool dvl_process();

    bool    get_instrument_velocity(instrument_reference_vel& data, reference_type type);
    bool    get_ship_velocity(ship_reference_vel& data, reference_type type);
    bool    get_earth_velocity(earth_reference_vel& data, reference_type type);
    bool    get_earth_distance(earth_reference_dist& data, reference_type type);

    int port_handle;
    int invalid_port_handle;
    char buffer[READ_BUF_SUZE];
    int program_exit, debug, test_mode;

    inst_vel_watermass      irefv_water;
    inst_vel_bottomtrack    irefv_bottom;

    ship_vel_watermass      srefv_water;
    ship_vel_bottomtrack    srefv_bottom;

    earth_vel_watermass     erefv_water;
    earth_vel_bottomtrack   erefv_bottom;

    earth_dist_watermass    erefd_water;
    earth_dist_bottomtrack  erefd_bottom;
};

#endif