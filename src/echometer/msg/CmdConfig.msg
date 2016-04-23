Header header
# Изменение режима работы эхолота
int8    workm       # Режим работы [1;6]
int8    range       # mm [1000; 100000]
int32   threshold   # sec [0.1; 3600]
float32 interval    # %%, altimeter threshold
int32   offset      # mm, offset of output altitude
int32   deadzone    # mm, minimal deadzone
int8    syncextern  # 0 - internal sync, 1 - external sync
int8    syncextmod  # 0 - falling 1 - rising