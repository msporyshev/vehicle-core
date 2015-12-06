# Номер камеры
uint8 camera_type

# Ширина кадра [пикс]
int32 width

# Высота кадра [пикс]
int32 height

# Количество каналов в изображении
int8 channels

# Байтовая последовательность cv::Mat.data
uint8[] frame