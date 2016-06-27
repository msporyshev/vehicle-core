#include "math_u.h"

#include <cmath>

namespace utils {

const double PI_DEG = 180.0;

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
    return normalize_angle(R);
}

//нормализация углов
float normalize_angle(float a)
{
    a = fmod(a + M_PI, 2 * M_PI);

    if (a < 0) {
        a += 2 * M_PI;
    }
    return a - M_PI;
}

float normalize_degree_angle(float a)
{
    a = fmod(a + PI_DEG, 2 * PI_DEG);
    if (a < 0) {
        a += 2 * PI_DEG;
    }

    return a - PI_DEG;
}

float calc_middle_head(float head1, float head2)
{
	normalize_angle(head1);
	normalize_angle(head2);
	float res = (head1 + head2) / 2;
	if (fabs(head1 - head2) > M_PI) res += M_PI;
	return normalize_angle(res);
}

double to_rad(double degree)
{
    return degree / PI_DEG * M_PI;
}

double to_deg(double radian)
{
    return radian / M_PI * PI_DEG;
}

} // namespace utils