#pragma once

#include <vector>

// ПИД-контроллер, позволяющий решить математическую задачу регулирования
// использует идеальную форму ПИД-регулирования

class PIDController
{
public:
    PIDController(double kp, double ki, double kd);
    ~PIDController();

    // возвращает величину тяги при заданных невязке и производной невязки
    // может вернуть величину, по модулю большую 1
    double update(double err, double err_d);

    // возвращает величину тяги при заданной невязкке
    // может вернуть величину, по модулю большую 1
    // производная невязки вычисляется численно
    double update(double err);
private:
    double kp;
    double ki;
    double kd;

    double last_err;
    double last_time;
    double integral;
};
