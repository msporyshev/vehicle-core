#pragma once

#include "motion_client.h"

class RobosubConfMotionClient : public MotionClient
{

public:
    RobosubConfMotionClient(ipc::Communicator& com);
    ~RobosubConfMotionClient();
    // прямое управление тягой по заданной оси
    // * value -- доля от максимальной тяги: [-1; 1]
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void thrust_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    void thrust_pitch(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_heading(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // отключение всех регуляторов
    void unfix_all();

    // управление курсом
    // * value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    // * timeout -- максимальное время работы регулятора в секундах
    // * kp, ki, kd -- коэффициенты пид-регулятора
    // * wm -- режим ожидания команды
    void fix_heading(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // поворот по курсу на заданное количество радиан
    // * bearing -- значение в радианах, на которое требуется изменить курс
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_right(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление дифферентом
    // * value -- значение дифферента в радианах, положительное направление соответствует повороту носа вверх,
    //   0 -- горизонтальное положение
    // * timeout -- максимальное время работы регулятора в секундах
    // * kp, ki, kd -- коэффициенты пид-регулятора
    // * wm -- режим ожидания команды
    void fix_pitch(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление глубиной
    // * value -- значение глубины в метрах, положительное направление вниз, 0 соответствует поверхности
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    // * kp, ki, kd -- коэффициенты пид-регулятора
    void fix_depth(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // изменение глубины на заданное количество метров
    // * value -- значение в метрах, на которое требуется изменить глубину
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_down(double depth, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_up(double depth, double timeout, WaitMode wm = WaitMode::WAIT);

    // стабилизация положения в горизонтальной плоскости в глобальной системе координат
    // * value -- двумерная точка, описывающая положение.
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * fwd_kp, fwd_ki, fwd_kd -- коэффициенты пид-регулятора продольного движения
    // * side_kp, side_ki, side_kd -- коэффициенты пид-регулятора поперечного движения
    // * wm -- режим ожидания команды
    void fix_position(Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    // движение строго по заданным направлениям на необходимое расстояние
    // выход к точке осуществляется в HOVER-режиме
    // * value -- двумерная точка расстояний по оси x и y:
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);

};
