#pragma once

struct DynamicParameters
{
    DynamicParameters(double delta_t, double t, double ax, double ay, double vx_prev = 0, 
        double vy_prev = 0) :
    delta_t(delta_t),
    t(t),
    ax(ax),
    ay(ay)
    {
        vx = ax * delta_t - vx_prev;
        vy = ay * delta_t - vy_prev;
    }

    double calc_vx(double vx_prev)
    {
        return this->vx = ax * delta_t - vx_prev;
    }
    
    double calc_vy(double vy_prev)
    {
       return this->vy = ay * delta_t - vy_prev;
    }

    double delta_t; ///< Время, прошедшее между получением текущего и предыдущего измерений
    double t; ///< Время получения данного измерения
    double vx; ///< Значение скорости по оси Х
    double vy; ///< Значение скорости по оси Y
    double ax; ///< Значение ускорение по оси  Х
    double ay; ///< Значение ускорение по оси Y
};
