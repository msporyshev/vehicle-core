#include "math_u.h"

float norm(float x, float y)
{
    return sqrt(x*x  + y*y);
}

/*
вычисление курса (от севера в рад по часовой) из точки 1 в точку 2
при этом берётся обычный декартов угол потому что система фоссеновская
*/
float kurs_point1_to_point2(float x1, float y1, float x2, float y2)
{
    return ((x2-x1 == 0) && (y2-y1 == 0)) ? 0 : atan2(y2-y1, x2-x1); 
}

//сложение углов
float add_angles(float fi1, float fi2)
{
    float R;
    R = fi1 + fi2;
    while (R > M_PI) R -= 2 * M_PI;
    while (R < -M_PI) R += 2 * M_PI;
    return R;
}

//нормализация углов
float normalize_angle(float a)
{
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

float calc_middle_head(float head1, float head2)
{
	normalize_angle(head1);
	normalize_angle(head2);
	float res = (head1 + head2) / 2;
	if (fabs(head1 - head2) > M_PI) res += M_PI;
	return normalize_angle(res);
}

double degree_to_radian(double degree)
{
    return degree / 180.0 * M_PI;
}

double radian_to_degree(double radian)
{
    return radian / M_PI * 180.0;
}

