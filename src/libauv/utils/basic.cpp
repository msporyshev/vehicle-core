#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "basic.h"

// Вывод в STDOUT маладших бит числа x (начиная с 0 и заканчивая num-1)
void print_bits(int x, int num)
{
	int i;
	for (i = num-1; i >= 0; i--)
	{
		printf("%d", (x & (1 << i)) ? 1 : 0);
	}
}

#ifndef __cplusplus
// Вывод в STDOUT в шестнадцатеричном и двоичном представлении
// содержимого num ячеек памяти, начиная с адреса addr
void print_memory(void *addr, int num)
{
	int i, j;
	unsigned char c, c1;
	char str[10];
	str[4] = 0x20;
	str[9] = 0x00;
	for (i = 0; i < num; i++)
	{
		c = c1 = *(unsigned char*)addr;
		for (j = 8; j >= 0; j--)
		{
			if (j == 4)
				continue;
			str[j] = 0x30 + (c & 0x01);
			c >>= 1;
		}
		printf("[%08X] = 0x%02X ( %s )\n", (int)addr, c1, str);
		addr++;
	}
}
#endif

// Возвращает текущее время c миллисекундной точностью в формате double
double timestamp()
{
	struct timeb timebuf;
	ftime(&timebuf);
	return ( (double)timebuf.time + (double)timebuf.millitm / 1000 );
}

void ctts()
{
	struct tm* ti;
	char buffer[16];
	struct timeb timebuf;
	ftime(&timebuf);

	ti = localtime(&timebuf.time);
    strftime(buffer, 16-1, "%H:%M:%S", ti);
    int h = timebuf.millitm;
    printf("%s.%d%d%d\t", buffer, h/100, (h/10)%10, h%10);
}

void print_date()
{
    time_t rt;
    struct tm * ti;

    time (&rt);
    ti = localtime (&rt);
    printf ("%d/%d/%d\t", (*ti).tm_mday, (*ti).tm_mon+1, (*ti).tm_year+1900);
}

void turn_output_buffering_off()
{
    int flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
    flags |= O_SYNC;
    fcntl(STDOUT_FILENO, F_SETFL, flags);

    int flags2 = fcntl(STDERR_FILENO, F_GETFL, 0);
    flags2 |= O_SYNC;
    fcntl(STDERR_FILENO, F_SETFL, flags2);
}

int get_power_of_two(int n)
{
    int power = 0;
    while (n != 1) {
        n /= 2;
        power++;
    }

    return power;
}
