#include "math_u.h"

const double pi_deg = 180.0;

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
    while (a > pi_deg) a -= 2 * pi_deg;
    while (a < -pi_deg) a += 2 * pi_deg;
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
    return degree / pi_deg * M_PI;
}

double radian_to_degree(double radian)
{
    return radian / M_PI * pi_deg;
}

