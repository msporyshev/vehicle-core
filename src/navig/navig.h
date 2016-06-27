#include <cmath>

#include <utils/math_u.h>

using namespace utils;

inline void integrate(double& value, double deriv, double dt)
{
    value += deriv * dt;
}

// Интеграл вектора в плоскости аппарата
template <class VecPlane1, class VecPlane2>
void integrate_plane(VecPlane1& value, VecPlane2 deriv, double dt)
{
    integrate(value.right, deriv.acc_y, dt); // ну пользуемся, раз уж написали =)
    integrate(value.forward, deriv.acc_x, dt);
}

// Интеграл на плоскости в относительной системе координат
template <class Position, class Velocity>
void integrate_local(Position& pos, Velocity v, double heading, double dt)
{
    double s = sin(to_rad(heading));
    double c = cos(to_rad(heading));
    integrate(pos.east, v.forward * s + v.right * c, dt);
    integrate(pos.north, v.forward * c - v.right * s, dt);
}


template <class T>
void update_age_info(T& age_info, double age, double age_max)
{
    age_info.age = age;
    age_info.fresh = (age < age_max);
    // Зачем фреш? если можно if unfresh_count

    // гугл транслейт даже не смог перевести unfresh
    age_info.unfresh_count += !age_info.fresh; // эта штука имеет смысл, только если мы по ссылке получаем age_info. В предыдущем варианте мы создавали новый.

    // Оставил с намерением выпилить :)
    // Хочется здесь все таки хранить сколько времени оно несвежее. И назвать типа time_not_fresh.
    // ну или еще что-то хранить, но че-то эти разы -- плохая характеристика, от частоты работы модуля зависит.
    // А время -- нет
}

// Функция расчета обоих компонент скорости (velocity)
// с учетом убегающей скорости по ускорениям (acc_vel)
// и скорости доплера (dvl_vel)
template <class Velocity, class DvlVelocity, class AccelVelocity>
bool try_get_velocity(
        Velocity& velocity,
        bool dvl_fresh,
        DvlVelocity dvl_vel,
        bool acc_fresh,
        AccelVelocity acc_vel
)
{
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


/*
  // ЗАГОТОВКА №1 //
  // Усреднение с затуханием без dt //

// Функция одной компоненты скорости
// с учетом убегающей скорости по ускорениям (acc_vel)
// и скорости доплера (dvl_vel)
double velocity_fusion(     // Возвращаем новую скорость по данной компоненте
    double &err,            // Отсюда берем старую и сюда записываем накопленную ошибку
    double  acc_vel,        // Скорость по ускорениям
    double  dvl_vel,        // Скорость по доплеру
    double  attenuation,    // Константа затухания подгонки под тренд доплера
    double  dt)             // Время с прошедшего вызова
{
    err = attenuation_dt * (acc_vel - dvl_vel) + (1.0 - attenuation) * err;
    return acc_vel - err;
}

// Функция расчета обоих компонент скорости (velocity)
// с учетом убегающей скорости по ускорениям (acc_vel)
// и скорости доплера (dvl_vel)
template <class T1, class T2, class T3>
bool velocity_accel_dvl(    // Обновляем флаг расчета скорости хоть по чем-нибудь
    T1    &velocity,        // Отсюда берем старую и сюда записываем новую скорость
    bool   acc_fresh,       // Флаг свежести скорости по ускорениям
    T2     acc_vel,         // Скорость по ускорениям
    bool   dvl_fresh,       // Флаг свежести скорости по доплеру
    T3     dvl_vel,         // Скорость по доплеру
    double attenuation,     // Константа затухания подгонки под тренд доплера
    double dt)              // Время с прошедшего вызова
{
    static double err_right   = 0.0;    // Изначальная накопленная ошибка счисления по ускорениям
    static double err_forward = 0.0;    // обнулена по обеим координатам

    // Если есть и доплер и ускорение, то производим слияние данных с поправкой ошибки
    if(acc_fresh && dvl_fresh) {
        velocity.right   = velocity_fusion(err_right,   acc_vel.right,   dvl_vel.right,   attenuation, dt);
        velocity.forward = velocity_fusion(err_forward, acc_vel.forward, dvl_vel.forward, attenuation, dt);
        return true;
    }
    // Если только ускорение, то возвращаем скорость по укорениям со старой поправкой на накопленную ошибку
    if(acc_fresh) {
        velocity.right   = acc_vel.right   - err_right;
        velocity.forward = acc_vel.forward - err_forward;
        return true;
    }
    // Если только ДОПЛЕР, то возвращаем скорость по доплеру
    if(dvl_fresh) {
        velocity.right   = dvl_vel.right;
        velocity.forward = dvl_vel.forward;
        return true;
    }
    // Если ничего нет, то выходим с флагом невозможности обновления данных
    return false;
}
*/

/*
  // ЗАГОТОВКА №2 //
  // Усреднение с затуханием по dt //

// ЗАГОТОВКА функции одной компоненты скорости
// с учетом убегающей скорости по ускорениям (acc_vel)
// и скорости доплера (dvl_vel)
void err_dt(        // Возвращаем новую скорость по данной компоненте
    double &w,      // Отсюда берем старую и сюда записываем накопленную ошибку
    double &x,      // Величина параметра
    double  x_new,  // Старая величина параметра
    double  k,      // Коэффициент затухания
    double  dt)     // Время с прошедшего вызова
{
    double k_dt = pow(k, dt);                           // Затухший коэффициент
    double w_last = w;                                  // Запоминаем старое значение веса
    w = k_dt * w_last + (1 - k);                        // Новый вес
    if(w > 0) {                                         // Если новый вес больше нуля,
        x = (k_dt * w_last * x + (1 - k) * x_new) / w;  // То поправляем величину
    }
}

// ЗАГОТОВКА Функция расчета обоих компонент скорости (velocity)
// с учетом убегающей скорости по ускорениям (acc_vel)
// и скорости доплера (dvl_vel)
template <class T1, class T2, class T3>
bool velocity_accel_dvl(    // Обновляем флаг расчета скорости хоть по чем-нибудь
    T1    &velocity,        // Отсюда берем старую и сюда записываем новую скорость
    bool   acc_fresh,       // Флаг свежести скорости по ускорениям
    T2     acc_vel,         // Скорость по ускорениям
    bool   dvl_fresh,       // Флаг свежести скорости по доплеру
    T3     dvl_vel,         // Скорость по доплеру
    double attenuation,     // Константа затухания подгонки под тренд доплера
    double dt)              // Время с прошедшего вызова
{
    static double weight      = 0.0;    // Накопленный вес
    static double err_right   = 0.0;    // Изначальная накопленная ошибка счисления по ускорениям
    static double err_forward = 0.0;    // обнулена по обеим координатам

    // Если ускорение, то возвращаем скорость по укорениям
    if(acc_fresh) {
        if(dvl_fresh) {     // Если есть еще и доплер, то вычисляем поправку накопившейся ошибки
            err_dt(weight, err_right,   acc_vel.right   - dvl_vel.right,   attenuation, dt);
            err_dt(weight, err_forward, acc_vel.forward - dvl_vel.forward, attenuation, dt);
        }
        velocity.right   = acc_vel.right   - err_right;
        velocity.forward = acc_vel.forward - err_forward;
        return true;
    }
    // Если только ДОПЛЕР, то возвращаем скорость по доплеру
    if(dvl_fresh) {
        velocity.right   = dvl_vel.right;
        velocity.forward = dvl_vel.forward;
        return true;
    }
    // Если ничего нет, то выходим с флагом невозможности обновления данных
    return false;
}
*/