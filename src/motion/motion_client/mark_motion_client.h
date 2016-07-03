#pragma once

#include "motion_client.h"

class MarkMotionClient : public MotionClient
{
public:

    /**
    Прямое управление тягой по продольной оси
    \param[in] value -- доля от максимальной тяги: [-1; 1]
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void thrust_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Тяга по оси курса
    \param[in] value -- доля от максимальной тяги: [-1; 1]
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void thrust_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Тяга по оси дифферента
    \param[in] value -- доля от максимальной тяги: [-1; 1]
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void thrust_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление курсом
    \param[in] value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_heading(double heading, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Поворот по курсу на заданное количество радиан
    \param[in] bearing -- значение в радианах, на которое требуется изменить курс
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void turn_right(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление дифферентом
    \param[in] value -- значение дифферента в радианах, положительное направление соответствует повороту носа вверх, 0 -- горизонтальное положение
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_pitch(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Поворот по дифференту на заданное количество радиан
    \param[in] value -- значение в радианах, на которое требуется изменить дифферент
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void turn_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Стабилизация положения в горизонтальной плоскости в глобальной системе координат
    \param[in] value -- двумерная точка, описывающая положение. x -- вперед, в метрах, y -- вправо, в метрах,
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_position(libauv::Point2d value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Движение строго по заданным направлениям на необходимое расстояние
    выход к точке осуществляется в CRUISE-режиме
    \param[in] value -- двумерная точка расстояний по оси x и y: x -- вперед, в метрах, y -- вправо, в метрах,
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void move_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление продольной скоростью
    \param[in] value -- значение скорости, в метрах в секунду
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_velocity(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление вертикальным каналом на скорости с помощью дифферента
    \param[in] value -- значение вертикального канала
    \param[in] mode -- режим выбора вертикального канала
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_vert(double value, SpeedyVertMode mode, double timeout, WaitMode wm = WaitMode::WAIT);
};
