// Для добавления новой оси:
// в файле regul.cpp (около 240 строки) в init_robosub() рекомендуется добавить строчку:
// cfg_robosub.source_info[SOURCE_NAME_SURNAME] = nullso;

#ifndef SOURCES_H_
#define SOURCES_H_

#define SOURCE_STABILIZE_OFF "off" // Pls, use "push_stabilize_off(axes axis)" and "publish_queue_off()" instead;
#define SOURCE_DIRECT_CONTROL  "direct"
#define SOURCE_COMPASS_HEAD "compass_head"
#define SOURCE_COMPASS_PITCH "compass_pitch"
#define SOURCE_SENSOR_DEPTH "depth"
#define SOURCE_STRIPE_LAG "stripe_lag"
#define SOURCE_STRIPE_DEPTH "stripe_depth"
#define SOURCE_DVL_XY "dvl_xy"

#endif
