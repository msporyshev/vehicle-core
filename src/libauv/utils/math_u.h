#pragma once

#define _USE_MATH_DEFINES

#include <math.h>


namespace utils {

#define DEG_to_R_ (M_PI / 180)
#define R_to_DEG_ (180 / M_PI)

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

// нормализация углов, заданных в градусах
float normalize_degree_angle(float a);

float degree_angle_diff(float a, float b);

float calc_middle_head(float head1, float head2);

double to_rad(double degree);
double to_deg(double radian);

} // namespace utils
