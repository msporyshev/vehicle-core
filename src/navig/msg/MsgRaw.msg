
navig/MsgAgeInfo dvl_down
navig/MsgAgeInfo supervisor_depth      # Служебка глубины и скорости изменения глубины от супервизора
navig/MsgAgeInfo dvl_plane_velocity    # Служебка вектора скорости (вперед, вправо) по доплеру
navig/MsgAgeInfo compass_angle         # Служебка курса, дифферента и крена от компаса
navig/MsgAgeInfo compass_angle_rate    # Служебка скорости курса, дифферента и крена от компаса
navig/MsgAgeInfo compass_acceleration  # Служебка ускорений вперед, вправо и вниз от компаса

bool velocity_flag             # Флаг возможности расчета скорости хоть по чем-нибудь
bool position_flag             # Флаг возможности расчета положения хоть по чем-нибудь

# Служебные отладочные скорости
navig/MsgPlaneVelocity velocity_acc
navig/MsgPlaneVelocity velocity

# Служебные отладочные координаты
navig/MsgLocalPosition position_acc      # Локальные координаты по интегрированию ускорений и курсу компаса
navig/MsgLocalPosition position_dvl      # Локальные координаты по доплеру и курсу компаса
navig/MsgLocalPosition position