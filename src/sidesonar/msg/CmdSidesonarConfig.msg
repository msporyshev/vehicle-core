# Длина импульса [число]. Настоящая длина импульса [мкс]: pulse_legth[число] * 2,5[мкс/число]
int8  pulse_legth

# Размер скан-строк. На один канал. data_size = (ширина луча[м] * adc_data_rate) / 1500(м/с)
int16 data_size

# Режим ВАРУ. 0 - -10dB (low); 1 - 0dB (normal); 2 - +10dB (high)
int8  tvg_mode

# Работающие каналы
# 0 - left|right channels turn off; 1 - left channel turn on;
# 2 - right channel turn on; 3 - left|right channels turn on
int8  channels

# Частота работы
# 10 - 100 kHz; 40 - 400 kHz; 90 - 900 kHz; 120 - 1250 kHz 
int8  work_frequency

# Режим тестирования
# 0 - normal; 1 - data set 1; 2 - data set 2; 3 - data set 3
int8  test_mode

# Усиление каналов
# 0 - Left normal, Right - normal gain; 1 - Left high, Right - normal gain
# 2 - Left normal, Right - high gain; 3 - Left high, Right - high gain; 
int8  channel_gain

# Частота оцифровки
# 1 - 50 ksamples/sec per channel; 2 - 25 ksamples/sec; 5 - 10 ksamples/sec
int8  adc_data_rate