//==============================================================================
//
// Name        : mti_compass_calibration_data_store.c
// Author      : Fedor Dubrovin
// Description : MTi compass (Xsens Technologies B.V.) calibration data store
// Version     : 001
// Modified    : 2012-08-25 13-00
//
//==============================================================================


// Идентификатор программы
#define TASK_NAME ( "mti_compass_calibration_data_store" )

// Плохой файловый дескриптор
#define INVALID_FILE_DESCRIPTOR ( -1 )

//------------------------------------------------------------------------------

#include <stdio.h>              // Standard Input / Output
#include <stdlib.h>             // General Utilities
#include <string.h>             // String Handling
#include <unistd.h>             // Symbolic Constants
#include <stdarg.h>             // Variable Arguments
#include <fcntl.h>              // File Control Operations
#include <termios.h>            // General Terminal Interface
#include <sys/ioctl.h>          //

//------------------------------------------------------------------------------

#include "utils/basic.h"              // Общие функции

//------------------------------------------------------------------------------
// Команды управления компасом MTi XSens

unsigned char GoToConfig[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
unsigned char Reset[] = {0xFA, 0xFF, 0x40, 0x00, 0xC1};


//==============================================================================
//
// Работа с COM-портом (открытие, настройка, закрытие)
//
//==============================================================================


//------------------------------------------------------------------------------
//
// Открытие COM-порта и его настройка для работы с инерциальным
// измерительным модулем MTi (Xsens Technologies B.V.)
//
//------------------------------------------------------------------------------
int MTI_COM_open ( const char *COM_name, int baudrate )
{
  // Файловый дескриптор для COM-порта
  int fd;

  // Открытие COM-порта
  fd = open ( COM_name, O_RDWR | O_NOCTTY | O_NDELAY );

  // Результат открытия COM-порта
  if ( fd == INVALID_FILE_DESCRIPTOR )
  {
    printf ( "\nНе удалось открыть COM-порт %s.\n\n", COM_name );
    return ( INVALID_FILE_DESCRIPTOR );
  }
  else
    printf ( "\nCOM-порт %s открыт.\n\n", COM_name );

  //----------------------------------------------------------------------------
  // Настройка COM-порта

  // Структура с настройками COM-порта
  struct termios options;

  // Получение текущих настроек COM-порта
  if ( tcgetattr ( fd , &options ) != 0 )
  {
    puts ( "\nНе удалось получить текущие настройки COM-порта.\n\nCOM-порт закрыт.\n" );
    close ( fd );
    return ( INVALID_FILE_DESCRIPTOR );
  }

  // Определение скорости обмена
  switch ( baudrate )
  {
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
      puts ( "\nНе удалось настроить COM-порт для работы на требуемой скорости.\n\nCOM-порт закрыт.\n" );
      close ( fd );
      return ( INVALID_FILE_DESCRIPTOR );
  }

  // Установка скорости работы COM-порта
  if ( cfsetspeed( &options, baudrate ) != 0 )
  {
    puts ( "\nНе удалось настроить COM-порт для работы на требуемой скорости.\n\nCOM-порт закрыт.\n" );
    close ( fd );
    return ( INVALID_FILE_DESCRIPTOR );
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

  options.c_cflag |= CSTOPB;    // Два стоповых бита
  //options.c_cflag &= ~CSTOPB;   // Один стоповый бит

  //options.c_cflag |= CRTSCTS;   // Включение аппаратного управления потоком
  options.c_cflag &= ~CRTSCTS;  // Отключение аппаратного управления потоком

  // Опции ввода (c_iflag)

  //options.c_iflag |= ( IXON | IXOFF | IXANY );  // Включение программного управления потоком
  options.c_iflag &= ~( IXON | IXOFF | IXANY ); // Отключение программного управления потоком

  options.c_iflag &= ~( INLCR | ICRNL | IGNCR | IUCLC | IMAXBEL | BRKINT );

  // Опции вывода (c_oflag)

  //options.c_oflag |= OPOST;    // Режим обработанного вывода
  options.c_oflag &= ~OPOST;    // Режим необработанного вывода. Остальные биты поля c_oflag игнорируются.

  // Локальные опции (c_lflag)

  options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );

  // Применение данных настроек COM-порта
  if ( tcsetattr ( fd , TCSANOW, &options ) != 0 )
  {
    puts ( "\nНе удалось настроить COM-порт.\n\nCOM-порт закрыт.\n" );
    close ( fd );
    return ( INVALID_FILE_DESCRIPTOR );
  }

  return ( fd );
}


//------------------------------------------------------------------------------
//
// Закрытие COM-порта
//
//------------------------------------------------------------------------------
void MTI_COM_close ( int *fd )
{
  if ( *fd != INVALID_FILE_DESCRIPTOR )
  {
    puts ( "\nCOM-порт закрыт.\n" );
    close( *fd );
    *fd = INVALID_FILE_DESCRIPTOR;
  }
  else
  {
    puts ( "\nПри закрытии COM-порта произошла ошибка. Возможно COM-порт уже закрыт.\n" );
  }
}


//------------------------------------------------------------------------------
//
// Вывод заголовка программы
//
//------------------------------------------------------------------------------
void print_program_header ( )
{
  puts ( "                                        " );
  puts ( "----------------------------------------" );
  puts ( "                                        " );
  puts ( "   MTi compass calibration data store   ");
  puts ( "                                        " );
  puts ( "             August 25, 2013            " );
  puts ( "                                        " );
  puts ( "----------------------------------------" );
  puts ( "                                        " );
}


//------------------------------------------------------------------------------
//
// Главная функция
//
//------------------------------------------------------------------------------
int main ( int argc, char *argv[] )
{
  // Файловый дескриптор для работы с COM-портом
  int COM_descriptor = INVALID_FILE_DESCRIPTOR;

  // Параметры COM-порта
  char COM_name[32] = "/dev/ttyS0";
  int COM_baudrate = 57600;

  // Файловый дескриптор для файла данных
  int fd_calibration = INVALID_FILE_DESCRIPTOR;

  // Буфер, в который считываются данные
  unsigned char buffer[8192];

  // Число реально прочитанных байт
  int n;

  // Переменная для разборщика параметров
  int opt;

  // Обработка параметров, переданных программе при запуске
  while ( ( opt = getopt ( argc, argv, "c:C:b:B:?hH" ) ) != -1 )
  {
    switch ( opt )
    {
      // Вывод справки по параметрам запуска драйвера
      case '?':
      case 'h':
      case 'H':
        // Вывод заголовка программы
        print_program_header ( );

        puts ( "Options description:" );
        puts ( "  c <arg>  - set COM-port name" );
        puts ( "  b <arg>  - set COM-port baudrate" );
        puts ( "" );
        return ( EXIT_SUCCESS );
        break;

      // Задание имени COM-порта
      case 'c':
      case 'C':
        strcpy ( COM_name, optarg );
        break;

      // Задание скорости работы COM-порта
      case 'b':
      case 'B':
        COM_baudrate = atoi ( optarg );
        break;

      default:
        puts ( "ERROR! Unsupported parameter" );
    }
  }

  // Вывод заголовка программы
  print_program_header ( );

  printf ( "COM_name = %s\n", COM_name );

  printf ( "COM_baudrate = %d\n", COM_baudrate );

  // Открытие COM-порта и его настройка
  COM_descriptor = MTI_COM_open ( COM_name, COM_baudrate );

  // Если не удалось открыть COM-порт
  if ( COM_descriptor == INVALID_FILE_DESCRIPTOR )
  {
    puts ( "ERROR: MTI_COM_open" );
    puts ( "Program was terminated" );

    return ( EXIT_FAILURE );
  }

  puts ( "Start of calibration data store" );

  // Открытие файла с данными для калибровки компаса
  fd_calibration = open ( "mfmResults.bin", O_RDONLY );

  if ( fd_calibration == INVALID_FILE_DESCRIPTOR )
  {
    puts ( "ERROR: Calibration file mfmResults.bin is absent" );
    puts ( "Program was terminated" );

    // Закрытие COM-порта
    MTI_COM_close ( &COM_descriptor );

    return ( EXIT_FAILURE );
  }

  puts ( "Calibration file mfmResults.bin was opened" );

  n = read ( fd_calibration, buffer, 65 );

  if ( n != 65 )
  {
    puts ( "ERROR: Calibration file mfmResults.bin reading" );

    close ( fd_calibration );
    return ( EXIT_FAILURE );
  }

  puts ( "Calibration file mfmResults.bin reading: OK" );

  n = write ( COM_descriptor, GoToConfig, sizeof ( GoToConfig ) );
  puts ( "GoToConfig" );
  usleep ( 500000 );

  n = write ( COM_descriptor, buffer, 65 );
  puts ( "StoreCalibrationData" );
  usleep ( 500000 );

  if ( n != 65 )
  {
    puts ( "ERROR: Calibration data writing error" );

    // Закрытие COM-порта
    MTI_COM_close ( &COM_descriptor );

    return (EXIT_FAILURE);
  }

  puts ( "Calibration was successfully finished" );

  close ( fd_calibration );
  puts ( "Calibration file mfmResults.bin was closed" );

  // Закрытие COM-порта
  MTI_COM_close ( &COM_descriptor );

  return ( EXIT_SUCCESS );
}

