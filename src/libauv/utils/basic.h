#pragma once

//Модуль с общими вспомогательными функциями

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h> // Функции времени

#include <sys/timeb.h> // Форматы для ftime

#ifndef _WIN32
#include "unistd.h"
#include "fcntl.h"
#endif

// Вывод в STDOUT маладших бит числа x (начиная с 0 и заканчивая num-1)
void print_bits(int x, int num);

#ifndef __cplusplus
// Вывод в STDOUT в шестнадцатеричном и двоичном представлении
// содержимого num ячеек памяти, начиная с адреса addr
void print_memory(void *addr, int num);

#endif

// Возвращает текущее время c миллисекундной точностью в формате double
double timestamp();

void ctts();

void print_date();

void turn_output_buffering_off();

int get_power_of_two(int n);


