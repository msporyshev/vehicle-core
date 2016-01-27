#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#define _USE_MATH_DEFINES

#include <math.h>

#define GR_to_R_ (M_PI / 180)
#define R_to_GR_ (180 / M_PI)

float norm(float x, float y);

/*
вычисление курса (от севера в рад по часовой) из точки 1 в точку 2
при этом берётся обычный декартов угол потому что система фоссеновская
*/
float kurs_point1_to_point2(float x1, float y1, float x2, float y2);

//сложение углов
float add_angles(float fi1, float fi2);

//нормализация углов
float normalize_angle(float a);

float calc_middle_head(float head1, float head2);

double degree_to_radian(double degree);
double radian_to_degree(double degree);

#endif
