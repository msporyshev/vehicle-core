Header header

# corrections type
# INVALID      = 0
# FIX          = 1
# DIFFERENTIAL = 2
# SENSITIVE    = 3
# RTKIN        = 4
# FLOAT_RTK    = 5
# ESTIMATED    = 6
# MANUAL       = 7
# SIMULATION   = 8
int32 type

#Time elapsed (in seconds) since last DGPS corrections
float64 dgps_time_elasped 

# SID of DPGS station
int32 dgps_sid