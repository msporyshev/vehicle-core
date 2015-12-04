#Послать сообщение data длиной len на ящик id.
#len должна быть в пределах 1-8 (размер сообщения в CAN).

uint32 version
float64 time
int32 id
int8[] data

#ifdef DELAY_DEBUG
#float64 tcu_sent_time;
#float64 regul_recieved_time;
#float64 regul_sent_time;
#float64 navig_recieved_time;
#float64 navig_sent_time;
#float64 depth_recieved_time;
#float64 depth_sent_time;
#float64 dvl_recieved_time;
#float64 dvl_sent_time;
#float64 compass_recieved_time;
#float64 compass_sent_time;
#endif
