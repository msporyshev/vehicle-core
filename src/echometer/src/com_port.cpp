#include "echometer/com_port.h"

int fd;       // Дескриптор COM-порта
void* handler;

int open_com_port ( const char *COM_name, int baudrate )
{
    // Открытие COM-порта
    fd = open ( COM_name, O_RDWR | O_NOCTTY | O_NDELAY );

    // Результат открытия COM-порта
    if ( fd == INVALID_FILE_DESCRIPTOR ) {
        printf ( "Can not open COM-port %s.\n", COM_name );
        return 1;
    } else {
        printf ( "COM-port %s is open.\n", COM_name );
    }
    //----------------------------------------------------------------------------
    // Настройка COM-порта

    struct termios options;       // Структура с настройками COM-порта

    // Получение текущих настроек COM-порта
    if ( tcgetattr ( fd , &options ) != 0 ) {
        puts ( "Can not get current COM-port settings. COM-port will be close.\n" );
        close ( fd );
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

    // Управляющие опции (c_cflag)
    options.c_cflag |= ( CLOCAL | CREAD );        // Обязательные опции

    options.c_cflag &= ~CSIZE;    // Маскирование битов размера симвоов
    //options.c_cflag |= CS5;       // 5 бит данных
    //options.c_cflag |= CS6;       // 6 бит данных
    //options.c_cflag |= CS7;       // 7 бит данных
    options.c_cflag |= CS8;       // 8 бит данных

    //options.c_cflag |= PARENB;    // Использовать бит четности
    options.c_cflag &= ~PARENB;   // Не использовать бит четности

    //options.c_cflag |= PARODD;    // Проверка на нечетность
    //options.c_cflag &= ~PARODD;   // Проверка на четность

    //!!!!options.c_cflag |= CSTOPB;    // Два стоповых бита
    options.c_cflag &= ~CSTOPB;   // Один стоповый бит

    //options.c_cflag |= CRTSCTS;   // Включение аппаратного управления потоком
    options.c_cflag &= ~CRTSCTS;  // Отключение аппаратного управления потоком

    // Опции ввода (c_iflag)

    //options.c_iflag |= ( IXON | IXOFF | IXANY );  // Включение программного управления потоком
    options.c_iflag &= ~( IXON | IXOFF | IXANY ); // Отключение программного управления потоком

    options.c_iflag &= ~( INLCR | ICRNL | IGNCR | IUCLC | IMAXBEL | BRKINT ); //INLCR | ICRNL ??

    // Опции вывода (c_oflag)

    //options.c_oflag |= OPOST;    // Режим обработанного вывода
    options.c_oflag &= ~OPOST;    // Режим необработанного вывода. Остальные биты поля c_oflag игнорируются.

    // Локальные опции (c_lflag)

    options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG ); //ECHO - зачем? FLUSHO  Output being flushed
    //options.c_lflag |= (ICANON | ECHO | ECHOE); может стоит лучше так?

    //!!! options.c_cc[VMIN]  = 0;
    //!!! options.c_cc[VTIME] = 10;

    // Применение данных настроек COM-порта
    if ( tcsetattr ( fd , TCSANOW, &options ) != 0 ) {
        puts ( "Can not configure COM-port. COM-port will be close.\n" );
        close ( fd );
        return 1;
    }

    return 0;
}

bool close_com_port ()
{
    if ( fd != INVALID_FILE_DESCRIPTOR ) {
        puts ( "COM-port closed.\n" );
        close( fd );
        fd = INVALID_FILE_DESCRIPTOR;
        return 0;
    }
    else {
        puts ( "Error while closing. May be already closed?\n" );
        return 1;
    }
}

void set_handler(void (*handler) (int))
{
    struct sigaction saio;
    
    sigset_t   set;
    sigemptyset(&set);
    sigaddset(&set, SIGIO); 
    
    saio.sa_handler = handler;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL; 
    saio.sa_mask = set;

    sigaction(SIGIO, &saio, NULL);
    fcntl(fd, F_SETOWN, getpid());
    fcntl(fd, F_SETFL, FNDELAY | FASYNC);
}

void send_to_com(const char* buf, int count)
{
    int actual_len;
    char message[50];
    if(count != 0) {
        actual_len = sprintf(message, "%s\r", buf);
        printf("Send on device: ");
        for(int j = 0; j < actual_len; j++)
            printf ( "%c", message[j] );
        printf("Sended %d bytes\n", actual_len);
        int act_send = write (fd, message, actual_len);
        printf("Sending over. Sending %d bytes\n", act_send);
    }
}

#define BUFF_SIZE 2048
char buff[BUFF_SIZE];
int get_message ( char* buffer, int size)
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