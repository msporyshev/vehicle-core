#include "compass/x-sens/MTI.h"

unsigned char MTI::GoToConfig[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
int MTI::GoToConfig_size = 5;
unsigned char MTI::GoToMeasurement[] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
int MTI::GoToMeasurement_size = 5;
unsigned char MTI::Reset[] = {0xFA, 0xFF, 0x40, 0x00, 0xC1};
int MTI::Reset_size = 5;
unsigned char MTI::SetBaudrate57600[] = {0xFA, 0xFF, 0x18, 0x01, 0x04, 0xE4};
int MTI::SetBaudrate57600_size = 6;
unsigned char MTI::SetBaudrate38400[] = {0xFA, 0xFF, 0x18, 0x01, 0x05, 0xE3};
int MTI::SetBaudrate38400_size = 6;
unsigned char MTI::SetMeasurementMode[] = {0xFA, 0xFF, 0xD0, 0x02, 0x08, 0x0F, 0x18};
int MTI::SetMeasurementMode_size = 7;
unsigned char MTI::SetMeasurementSettings[] = {0xFA, 0xFF, 0xD2, 0x04, 0x80, 0x00, 0x00, 0x05, 0xA6};
int MTI::SetMeasurementSettings_size = 9;
unsigned char MTI::SetRawMode[] = {0xFA, 0xFF, 0xD0, 0x02, 0x40, 0x00, 0xEF};
int MTI::SetRawMode_size = 7;
unsigned char MTI::SetRawSettings[] = {0xFA, 0xFF, 0xD2, 0x04, 0x00, 0x00, 0x00, 0x01, 0x2A};
int MTI::SetRawSettings_size = 9;
unsigned char MTI::StoreXKFState[] = {0xFA, 0xFF, 0x8A, 0x00, 0x77};
int MTI::StoreXKFState_size = 5;
unsigned char MTI::SetOutputMode[] = {0xFA, 0xFF, 0xD0, 0x02, 0x40, 0x00, 0xEF};
int MTI::SetOutputMode_size = 7;
unsigned char MTI::SetOutputSettings[] = {0xFA, 0xFF, 0xD2, 0x04, 0x00, 0x00, 0x00, 0x01, 0x2A};
int MTI::SetOutputSettings_size = 9;
unsigned char MTI::SetOutputSkipfactor[] = {0xFA, 0xFF, 0xD4, 0x00, 0x04, 0x29};
int MTI::SetOutputSkipfactor_size = 6;

MTI::MTI()
{
    invalid_file_descriptor = -1;
    show_debug_messages = 0;
}

void MTI::mprintf (const char *format, ...)
{
    va_list argptr;

    if (show_debug_messages) {
        va_start (argptr, format);
        vprintf (format, argptr);
    }
}

void MTI::print_memory_table (unsigned char *addr, int num)
{
    int i;

    for (i = 0; i < num; i++) {
        if (i % 16 == 0)
            puts ("");
        printf ("%02X  ", *addr);
        addr++;
    }
    puts ("");
}

void MTI::print_memory_full_table (unsigned char *addr, int num)
{
    int i;

    printf ("\n\nMemory dump from adress %08X:\n\n", (size_t) addr);

    puts ("  Offset  | 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F ");
    puts ("----------|---------------------------------------------------------------");

    for (i = 1; i <= num; i++) {
        if (i % 16 == 1)
            printf (" %08X |", (unsigned int) (i - 1));

        printf (" %02X ", *addr);

        addr++;

        if (i % 16 == 0)
            puts ("");
    }
    puts ("\n");
}

int MTI::MTI_COM_open (const char *COM_name, int baudrate)
{
    int fd;       // Дескриптор COM-порта

    // Открытие COM-порта
    fd = open (COM_name, O_RDWR | O_NOCTTY | O_NDELAY);

    // Результат открытия COM-порта
    if (fd == invalid_file_descriptor) {
        printf ("\nНе удалось открыть COM-порт %s.\n\n", COM_name);
        return (invalid_file_descriptor);
    }
    else
        printf ("\nCOM-порт %s открыт.\n\n", COM_name);

    //----------------------------------------------------------------------------
    // Настройка COM-порта

    //fcntl (fd, F_SETFL, FNDELAY);
    //fcntl (fd, F_SETFL, 0);

    struct termios options;       // Структура с настройками COM-порта

    // Получение текущих настроек COM-порта
    if (tcgetattr (fd , &options)) {
        puts ("\nНе удалось получить текущие настройки COM-порта.\n\nCOM-порт закрыт.\n");
        close (fd);
        return (invalid_file_descriptor);
    }

    // Определение скорости обмена
    switch (baudrate) {
    case 300:          baudrate = B300; break;
    case 1200:          baudrate = B1200; break;
    case 2400:          baudrate = B2400; break;
    case 4800:          baudrate = B4800; break;
    case 9600:          baudrate = B9600; break;
    case 19200:         baudrate = B19200; break;
    case 38400:         baudrate = B38400; break;
    case 57600:         baudrate = B57600; break;
    case 115200:        baudrate = B115200; break;
    case 230400:        baudrate = B230400; break;
    case 460800:        baudrate = B460800; break;
    default:
        puts ("\nНе удалось настроить COM-порт для работы на требуемой скорости.\n\nCOM-порт закрыт.\n");
        close (fd);
        return (invalid_file_descriptor);
    }

    // Установка скорости работы COM-порта
    if (cfsetspeed(&options, baudrate)) {
        puts ("\nНе удалось настроить COM-порт для работы на требуемой скорости.\n\nCOM-порт закрыт.\n");
        close (fd);
        return (invalid_file_descriptor);
    }

    // Управляющие опции (c_cflag)

    options.c_cflag |= (CLOCAL | CREAD);        // Обязательные опции

    options.c_cflag &= ~CSIZE;    // Маскирование битов размера симвоов
    //options.c_cflag |= CS5;       // 5 бит данных
    //options.c_cflag |= CS6;       // 6 бит данных
    //options.c_cflag |= CS7;       // 7 бит данных
    options.c_cflag |= CS8;       // 8 бит данных

    //options.c_cflag |= PARENB;    // Использовать бит четности
    options.c_cflag &= ~PARENB;   // Не использовать бит четности

    //options.c_cflag |= PARODD;    // Проверка на нечетность
    //options.c_cflag &= ~PARODD;   // Проверка на четность

    options.c_cflag |= CSTOPB;    // Два стоповых бита
    //options.c_cflag &= ~CSTOPB;   // Один стоповый бит

    //options.c_cflag |= CRTSCTS;   // Включение аппаратного управления потоком
    options.c_cflag &= ~CRTSCTS;  // Отключение аппаратного управления потоком

    // Опции ввода (c_iflag)

    //options.c_iflag |= (IXON | IXOFF | IXANY);  // Включение программного управления потоком
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Отключение программного управления потоком

    options.c_iflag &= ~(INLCR | ICRNL | IGNCR | IUCLC | IMAXBEL | BRKINT);

    // Опции вывода (c_oflag)

    //options.c_oflag |= OPOST;    // Режим обработанного вывода
    options.c_oflag &= ~OPOST;    // Режим необработанного вывода. Остальные биты поля c_oflag игнорируются.

    // Локальные опции (c_lflag)

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Применение данных настроек COM-порта
    if (tcsetattr (fd , TCSANOW, &options)) {
        puts ("\nНе удалось настроить COM-порт.\n\nCOM-порт закрыт.\n");
        close (fd);
        return (invalid_file_descriptor);
    }

    return (fd);
}

void MTI::MTI_COM_close (int *fd)
{
    if (*fd != invalid_file_descriptor) {
        puts ("\nCOM-порт закрыт.\n");
        close(*fd);
        *fd = invalid_file_descriptor;
    }
    else {
        puts ("\nПри закрытии COM-порта произошла ошибка. Возможно COM-порт уже закрыт.\n");
    }
}

int MTI::package_decode (unsigned char *package, MTI_struct *MTI_data_pointer)
{
    union
    {
        unsigned char c[4];
        unsigned short int si[2];
        float f;
    } tmp;

    int size = package[3];        // Размер блока данных

    //printf ("Package size = %d\n", size);

    if (size == 59) {
        tmp.c[3] = package[4];
        tmp.c[2] = package[5];
        tmp.c[1] = package[6];
        tmp.c[0] = package[7];
        MTI_data_pointer->temp = tmp.f;      // Температура

        tmp.c[3] = package[8];
        tmp.c[2] = package[9];
        tmp.c[1] = package[10];
        tmp.c[0] = package[11];
        MTI_data_pointer->accX = tmp.f;      // Ускорение вперед

        tmp.c[3] = package[12];
        tmp.c[2] = package[13];
        tmp.c[1] = package[14];
        tmp.c[0] = package[15];
        MTI_data_pointer->accY = tmp.f;      // Ускорение вправо

        tmp.c[3] = package[16];
        tmp.c[2] = package[17];
        tmp.c[1] = package[18];
        tmp.c[0] = package[19];
        MTI_data_pointer->accZ = tmp.f;      // Ускорение вниз

        tmp.c[3] = package[20];
        tmp.c[2] = package[21];
        tmp.c[1] = package[22];
        tmp.c[0] = package[23];
        MTI_data_pointer->gyrX = tmp.f;      // Угловая скорость по крену

        tmp.c[3] = package[24];
        tmp.c[2] = package[25];
        tmp.c[1] = package[26];
        tmp.c[0] = package[27];
        MTI_data_pointer->gyrY = tmp.f;      // Угловая скорость по дифференту

        tmp.c[3] = package[28];
        tmp.c[2] = package[29];
        tmp.c[1] = package[30];
        tmp.c[0] = package[31];
        MTI_data_pointer->gyrZ = tmp.f;      // Угловая скорость по курсу

        tmp.c[3] = package[32];
        tmp.c[2] = package[33];
        tmp.c[1] = package[34];
        tmp.c[0] = package[35];
        MTI_data_pointer->magX = tmp.f;      // Магнитное поле - продольная составляющая

        tmp.c[3] = package[36];
        tmp.c[2] = package[37];
        tmp.c[1] = package[38];
        tmp.c[0] = package[39];
        MTI_data_pointer->magY = tmp.f;      // Магнитное поле - поперечная составляющая

        tmp.c[3] = package[40];
        tmp.c[2] = package[41];
        tmp.c[1] = package[42];
        tmp.c[0] = package[43];
        MTI_data_pointer->magZ = tmp.f;      // Магнитное поле - вертикальная составляющая

        tmp.c[3] = package[44];
        tmp.c[2] = package[45];
        tmp.c[1] = package[46];
        tmp.c[0] = package[47];
        MTI_data_pointer->roll = tmp.f;      // Крен

        tmp.c[3] = package[48];
        tmp.c[2] = package[49];
        tmp.c[1] = package[50];
        tmp.c[0] = package[51];
        MTI_data_pointer->pitch = tmp.f;     // Дифферент

        tmp.c[3] = package[52];
        tmp.c[2] = package[53];
        tmp.c[1] = package[54];
        tmp.c[0] = package[55];
        MTI_data_pointer->yaw = tmp.f;       // Курс

        tmp.c[1] = package[56];
        tmp.c[0] = package[57];
        MTI_data_pointer->Ain_1 = tmp.si[0]; // Канал 1 встроенного АЦП

        tmp.c[1] = package[58];
        tmp.c[0] = package[59];
        MTI_data_pointer->Ain_2 = tmp.si[0]; // Канал 2 встроенного АЦП

        tmp.c[0] = package[60];
        MTI_data_pointer->status = tmp.c[0]; // Код состояния

        tmp.c[1] = package[61];
        tmp.c[0] = package[62];
        MTI_data_pointer->TS = tmp.si[0];    // Счетчик сообщений

        return (0);
    }

    return (-1);
}

int MTI::read_buffer_block (int fd, unsigned char* buffer, int len)
{
    int qread;                    // Число успешно прочитанных байт

    int bad_count = 0;

    while (len > 0) { // Исправить логику! критичное место
        qread = read (fd, buffer, len);
        
        if (qread >= 0) {
            bad_count = 0;
            buffer += qread;
            len -= qread;
        } else {
            bad_count++;
            usleep(1);
            // ros::Duration(0.1).sleep();
            if(bad_count > 50) {
                // printf("Bad return\n");
                return (-1);
            }
        }
    }

    // printf("Good return\n");
    return (0);
}

int MTI::read_buffer_nonblock (int fd, unsigned char* buffer, int len)
{
    static int size = 0;          // Число байт, которые нужно прочитать
    static unsigned char *addr = NULL;     // Адресс текущего заполняемого байта
    int qread;                    // Число успешно прочитанных байт

    if (buffer == NULL) {
        size = 0;
        addr = NULL;
        return (-1);
    }

    if (size == 0) {
        size = len;
        addr = buffer;
    }

    qread = read (fd, addr, size);

    if (qread > 0) {
        size -= qread;
        addr += qread;

        if (size == 0) {
            addr = NULL;
            return (0);
        }
    }

    return (-1);
}

int MTI::read_buffer_part_nonblock (int fd, unsigned char *buffer, int size)
{
    return (read (fd, buffer, size));
}

int MTI::MTI_get_one_package_block (int fd, MTI_struct *MTI_data_pointer)
{
#define BUFFER_SIZE 1024                                    // Размер буфера
    static unsigned char buffer[BUFFER_SIZE];                   // Буфер
    int len, i, j;
    unsigned char CS;
    int status = 0;
    int bad_count_global = 0;
    int bad_count = 0;
    // printf("enter in one package block\n");

    ros::Time start = ros::Time::now();

    tcflush(fd, TCIOFLUSH);

    while (1) {
        i = 0;

        // printf("start cycle finding\n");
        if(bad_count_global > 5) {
            return (-1);
        }
        // Цикл ожидания 0xFA
        bad_count = 0;
        do {
            status = read_buffer_block (fd, &buffer[i], 1);
            bad_count++;
            if(bad_count > 80 || status < 0) {
                // printf("0xFA receiving break.\n");
                break;
            }
            // printf("data: %02X\n", buffer[i]);
        } while (buffer[i] != 0xFA);

        if (buffer[i] != 0xFA) {
            // printf("get 0xFA failed\n");
            bad_count_global++;
            continue;
        }
        // printf("get 0xFA\n");
        
        i++;

        // Следующий байт
        status = read_buffer_block (fd, &buffer[i], 1);
        if(status < 0) {
            // printf("cannot get data for 0xFF\n");
            bad_count_global++;
            continue;
        }

        // Если получен не 0xFF
        if (buffer[i] != 0xFF) {
            // printf("get 0xFF failed\n");
            bad_count_global++;
            continue;
        }
        // printf("get 0xFF!\n");
        i++;

        // Еще 2 байта (MID и LEN)
        status = read_buffer_block (fd, &buffer[i], 2);
        if(status < 0) {
            // printf("cannot get data for MID\n");
            bad_count_global++;
            continue;
        }
        // printf("Get MID!\n");
        i++;

        len = buffer[i];

        i++;

        // Блок данных + еще 1 байт (контрольная сумма)
        status = read_buffer_block (fd, &buffer[i], len + 1);
        if(status < 0) {
            // printf("cannot get data for DATA\n");
            bad_count_global++;
            continue;
        }

        i += len + 1;

        // Расчет контрольной суммы
        CS = 0;

        for (j = 1; j < i; j++)
            CS += buffer[j];

        // Проверка контрольной суммы
        if (CS != 0) {
            // mprintf ("Получен поврежденный пакет размером %i байт(а).\n", i);
            bad_count_global++;
            continue;
        }

        break;
    }

    //mprintf ("Получен корректный пакет размером %i байт(а).\n", i);
    //print_memory_table (buffer, i);

    // Проверка пакета на предмет соответствия типу MTData
    if (buffer[2] == 0x32) {
        // Обработка пакета MTData
        if (package_decode (buffer, MTI_data_pointer) == 0) {
            ros::Time finish = ros::Time::now();
            ROS_DEBUG_STREAM("Package handing time :" << (finish - start));
            return (0);
        }
    }

    return (-1);
}

int MTI::MTI_get_one_package_nonblock (int fd)
{
/*  #define BUFFER_SIZE 1024                                    // Размер буфера
  static unsigned char buffer[BUFFER_SIZE];                   // Буфер
  static state = 0;
  static i = 0;
  static qread = 0;
  int len, j;
  unsigned char CS;

  // Чтение пакета
  switch (state)
  {
    // Ожидание 0xFA
    case 0:
      do
      {
        if (read (fd, &buffer[i], 1) != 1)
          return (-1);
      }
      while (buffer[i] != 0xFA);

      state++;
      i++;

    // Получение 0xFF сразу после 0xFA
    case 1:
      if (read (fd, &buffer[i], 1) != 1)
          return (-1);

      // Если получен не 0xFF
      if (buffer[i] != 0xFF)
      {
        if (buffer[i] != 0xFA)
        {
          state = 0;
          i = 0;
        }
        return (-1);
      }

      state++;
      i++;

    // Получение 2 байт (MID и LEN)
    case 2:


    // Блок данных + еще 1 байт (контрольная сумма)
    case 3:

  }

  // Расчет контрольной суммы
  CS = 0;

  for (j = 1; j < i; j++)
  {
    CS += buffer[j];
  }

  // Проверка контрольной суммы
  if (CS)
  {
    printf ("Получен поврежденный пакет размером %i байт(а).\n", i);
    printf ("Контрольная сумма = %02X.\n", CS);
    printf ("--------------------------------------------\n");
    continue;
  }

  //printf ("Получен корректный пакет размером %i байт(а).\n", i);
  //print_memory_table (buffer, i);

  // Проверка пакета на предмет соответствия типу MTData
  if (buffer[2] == 0x32)
  {
    // Обработка пакета MTData
    package_decode (buffer);
  }

  return (0);


  while (1)
  {

    // Еще 2 байта (MID и LEN)
    read (fd, &buffer[i], 2);

    i++;

    len = buffer[i];

    i++;

    // Блок данных + еще 1 байт (контрольная сумма)
    read (fd, &buffer[i], len + 1);

    i += len + 1;
  }
*/
  return 0;
}

int MTI::MTI_send_package (int fd, const unsigned char *buffer, int len)
{
    static int written = 0;       // Число отправленных байт пакета ВООБЩЕ
    int q;                        // Число отправленных байт в данном вызове

    if (written == 0)
        mprintf ("Начало отправки компасу пакета длиной %d байт(а).\n", len);

    q = write (fd, &buffer[written], len - written);

    if (q < 0) {
        mprintf ("При отправке пакета возникла ошибка.\n");
        written = 0;
        return (-1);
    }
    else if (q == len - written) {
        mprintf ("Отправка компасу пакета завершена.\n");
        written = 0;
        return (0);
    } else {
        mprintf ("Передано %d байт(а).\n", q);
        written += q;
        return (-1);
    }
}

int MTI::MTI_get_package (int fd, unsigned char **buffer, int *len)
{
#define RING_BUFFER_SIZE 8192                           // Размер буфера
    static char ring_buffer[RING_BUFFER_SIZE];              // Буфер
    //  static unsigned short int start = 0;                    // Первая заполненная ячейка
    static unsigned short int index = 0;                    // Первая свободная ячейка
    //  static unsigned short int field = 0;                    // Число заполненных ячеек
    //  static unsigned short int vacant = RING_BUFFER_SIZE;    // Число свободных ячеек
    int n;                                                  // Число доступных для чтения байт
    //  int q;                                                  // Число читаемых байт
    int qread;                                              // Число успешно прочитанных байт

    // Получение числа доступных байт в приемном буфере COM-порта
    ioctl (fd, FIONREAD, &n);

    mprintf ("Число байт в приемном буфере COM-порта = %d\n", n);

    /*

  if (n > vacant)
  {
    printf ("Переполнение!\n");
    return (-1);
  }
  else if (n <= vacant)
  {

  }

*/

    // Если данные помещаются целиком
    if (index + n <= RING_BUFFER_SIZE) {
        qread = read (fd, &ring_buffer[index], n);

        //print_memory_table (&ring_buffer[index], qread);

        if (qread > 0) {
            index += qread;

            if (index >= RING_BUFFER_SIZE)
                index = 0;
        }
    }
    else {
        // Чтение части данных
        qread = read (fd, &ring_buffer[index], RING_BUFFER_SIZE - index);

        //print_memory_table (&ring_buffer[index], qread);

        if (qread > 0)
            index += qread;

        if (index >= RING_BUFFER_SIZE) {
            index = 0;

            qread = read (fd, &ring_buffer[index], RING_BUFFER_SIZE - index);

            //print_memory_table (&ring_buffer[index], qread);

            if (qread > 0)
                index += qread;

            qread = read (fd, &ring_buffer[index], n - (RING_BUFFER_SIZE - index));

            //print_memory_table (&ring_buffer[index], qread);

            if (qread > 0)
                index += qread;
        }
    }

    // Анализ
    //

    //print_memory_table (ring_buffer, RING_BUFFER_SIZE);

    return (0);
}

unsigned char MTI::MTI_ComputeChecksum (unsigned char *buffer, int len)
{
    unsigned char CS = 0;

    while (len--)
        CS -= *(buffer++);

    return (CS);
}

void MTI::MTI_reset (int fd)
{
    write (fd, Reset, Reset_size);
    usleep (500000);
    // ros::Duration(0.5).sleep();
}

void MTI::MTI_init (int fd)
{
  write (fd, GoToConfig, GoToConfig_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  write (fd, SetMeasurementMode, SetMeasurementMode_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  write (fd, SetMeasurementSettings, SetMeasurementSettings_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  write (fd, SetBaudrate57600, SetBaudrate57600_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  // write (fd, SetBaudrate38400, SetBaudrate38400_size);
  // usleep (500000);

  write (fd, Reset, Reset_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();
}

void MTI::MTI_set_skipfactor (int fd)
{
    write (fd, GoToConfig, GoToConfig_size);
    usleep (500000);

    write (fd, SetOutputSkipfactor, SetOutputSettings_size);
    usleep (500000);

    write (fd, Reset, Reset_size);
    usleep (500000);
}


void MTI::MTI_store_filter (int fd)
{
  write (fd, GoToConfig, GoToConfig_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  write (fd, StoreXKFState, StoreXKFState_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();

  write (fd, GoToMeasurement, GoToMeasurement_size);
  usleep (500000);
  // ros::Duration(0.5).sleep();
}

void MTI::MTI_start_calibrate (int fd)
{
    write (fd, GoToConfig, GoToConfig_size);
    usleep (500000 );

    write (fd, SetOutputMode, SetOutputMode_size);
    usleep (500000 );

    write (fd, SetOutputSettings, SetOutputSettings_size);
    usleep (500000);
}

void MTI::MTI_stop_calibrate (int fd)
{
    write (fd, GoToConfig, GoToConfig_size);
    usleep (500000);
}
