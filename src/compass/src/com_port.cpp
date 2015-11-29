#include "compass/com_port.h"

#define BUF_SIZE 1000

int fd;       // Дескриптор COM-порта

int open_com_port ( const char *com_name, int baudrate )
{
  fd = open ( com_name, O_RDWR | O_NOCTTY | O_NDELAY ); //открываем неблокируемо
  // Результат открытия COM-порта
  if ( fd == INVALID_FILE_DESCRIPTOR ) {
    printf ( "Can not open COM-port %s.\n", com_name );
    return 1;
  }

  struct termios options;       // Структура с настройками COM-порта

  // Получение текущих настроек COM-порта
  if ( tcgetattr ( fd , &options ) != 0 ) {
    puts ( "Can not get current COM-port settings. COM-port will be close.\n" );
    close (fd);
    return 1;
  }

  // Определение скорости обмена
  switch ( baudrate ) {
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
      puts ( "Current baundrate is not correct. COM-port will be close.\n" );
      close ( fd );
      return 1;
  }

  // Установка скорости работы COM-порта
  if ( cfsetspeed( &options, baudrate ) != 0 ) {
    puts ( "Can not configure COM-port using current baundrate. COM-port will be close.\n" );
    close ( fd );
    return 1;
  }

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

    //options.c_cflag |= CSTOPB;    // Два стоповых бита
    options.c_cflag &= ~CSTOPB;   // Один стоповый бит

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
  if ( tcsetattr ( fd , TCSANOW, &options ) != 0 ) {
    puts ( "Can not configure COM-port. COM-port will be close.\n" );
    close ( fd );
    return 1;
  }
  
  //fcntl(fd, F_SETFD, fcntl(fd, F_GETFD) | O_NONBLOCK);

  return 0;
}

bool close_com_port ()
{
  if ( fd != INVALID_FILE_DESCRIPTOR ) {
    close( fd );
    fd = INVALID_FILE_DESCRIPTOR;
    return 0;
  } else {
    puts ( "Error while closing. Maybe already closed?\n" );
    return 1;
  }
}

void send_to_com(const char* buf, int count)
{
  // int actual_len;
  // char message[100];
  if(count != 0) {
    //actual_len = sprintf(message, "%s\r", buf);
    printf("Send on device: ");
    for(int j = 0; j < count; j++)
      printf ( "%d ", (int)buf[j] );
    printf("\n");
    
    printf("Sended %d bytes\n", count);
    int act_send = write (fd, buf, count);
    printf("Sending over. Sending %d bytes\n", act_send);
  }
}

#define BUFF_SIZE 2048
char buff[BUFF_SIZE];

int get_message(char* buffer, int size)
{
  int count_read = read (fd, buff, BUFF_SIZE);
  if(count_read >= 0) {
    if(size >= count_read)
      memcpy(buffer, buff, count_read);
    else
      memcpy(buffer, buff, size);
  }
  return count_read;
}