/**
\file
\brief Математические и служебные фукции навигации
*/

#include <cmath>
#include <utils/math_u.h>

using namespace utils;

/** Интегрирование методом прямоугольников
\param[in,out] value  Значение интеграла на предыдущем и последующем шагах
\param[in]     deriv  Производная величины
\param[in]     dt     Время интегрирования
*/
inline void integrate(double &value, double deriv, double dt)
{
    value += deriv * dt;
}

/** Интегрирование методом прямоугольников вектора в плоскости аппарата
\param[in,out] value  Значение вектора (right, forward) на предыдущем и последующем шагах
\param[in]     deriv  Производная вектора (right, forward)
\param[in]     dt     Время интегрирования
*/
template <class VecPlane1, class VecPlane2>
void integrate_plane(VecPlane1 &value, VecPlane2 deriv, double dt)
{
    integrate(value.right,   deriv.right,   dt);
    integrate(value.forward, deriv.forward, dt);
}

/** Интеграл на плоскости в относительной системе координат
\param[in,out] pos      Значение вектора координат (nort, east) на предыдущем и последующем шагах
\param[in]     vel      Значение вектора скорости (forward, right)
\param[in]     heading  Курс аппарата [град]
\param[in]     dt       Время интегрирования
*/
template <class Position, class Velocity>
void integrate_local(Position &pos, Velocity vel, double heading, double dt)
{
    double s = sin(to_rad(heading));
    double c = cos(to_rad(heading));
    integrate(pos.east,  vel.forward * s + vel.right * c, dt);
    integrate(pos.north, vel.forward * c - vel.right * s, dt);
}

/**
\brief Обновление служебных данных возраста сообщений
\param[in,out] info     Переменная, в которую сваливается служебка о приходящих данных
\param[in]     age      Возраст данных для отладки и выбора частоты работы навига/модулей
\param[in]     age_max  Максимальный возраст, выше которого убирается флаг свежести

Шаблонная функция обновляет:
- возраст данных (age);
- флаг свежести данных (age_max);
- счетчик устаревших данных (unfresh_count) В ПРОШЛОМ, т.е. сколько раз не могли породить сообщения с использованием этих данных.
*/
template <class T>
void update_age_info(T& info, double age, double age_max)
{
    info.age = age;
    info.fresh = (age < age_max);
    info.unfresh_count += !info.fresh;
}

/** \brief Объединение скоростей по ускорениям и доплеру

Функция расчета обоих компонент скорости (velocity)
с учетом убегающей скорости по ускорениям (acc_vel)
и скорости доплера (dvl_vel)
*/
template <class Velocity, class AccelVelocity, class DvlVelocity>
bool try_get_velocity(
    Velocity      &velocity,
    bool           acc_fresh,
    AccelVelocity  acc_vel,
    bool           dvl_fresh,
    DvlVelocity    dvl_vel
){
    static double err_right   = 0.0;
    static double err_forward = 0.0;

    if(dvl_fresh) {
        if(acc_fresh) {
            err_right   = acc_vel.right   - dvl_vel.right;
            err_forward = acc_vel.forward - dvl_vel.forward;
        }

        velocity.right   = dvl_vel.right;
        velocity.forward = dvl_vel.forward;
        return true;
    }

    if(acc_fresh) {
        velocity.right   = acc_vel.right   - err_right;
        velocity.forward = acc_vel.forward - err_forward;
        return true;
    }

    return false;
}
