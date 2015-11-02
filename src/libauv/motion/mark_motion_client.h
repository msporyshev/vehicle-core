#pragma once

#include "motion_client.h"

class MarkMotionClient : public MotionClient
{
public:

    // прямое управление тягой по продольной оси
    // * value -- доля от максимальной тяги: [-1; 1]
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void thrust_forward(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_backward(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // тяга по оси курса
    // * value -- доля от максимальной тяги: [-1; 1]
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void thrust_right(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_left(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // тяга по оси дифферента
    // * value -- доля от максимальной тяги: [-1; 1]
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void thrust_down(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_up(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // управление курсом
    // * value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_heading(float heading, float timeout, WaitMode wm = WaitMode::WAIT);

    // поворот по курсу на заданное количество радиан
    // * bearing -- значение в радианах, на которое требуется изменить курс
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_right(float bearing, float timeout, WaitMode wm = WaitMode::WAIT);
    void turn_left(float bearing, float timeout, WaitMode wm = WaitMode::WAIT);

    // управление дифферентом
    // * value -- значение дифферента в радианах, положительное направление соответствует повороту носа вверх,
    //   0 -- горизонтальное положение
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_pitch(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // поворот по дифференту на заданное количество радиан
    // * value -- значение в радианах, на которое требуется изменить дифферент
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_up(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void turn_down(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // стабилизация положения в горизонтальной плоскости в глобальной системе координат
    // * value -- двумерная точка, описывающая положение.
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_position(Point2d value, float timeout, WaitMode wm = WaitMode::WAIT);

    // движение строго по заданным направлениям на необходимое расстояние
    // выход к точке осуществляется в CRUISE-режиме
    // * value -- двумерная точка расстояний по оси x и y:
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_forward(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(float value, float timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // управление продольной скоростью
    // * value -- значение скорости, в метрах в секунду
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_velocity(float value, float timeout, WaitMode wm = WaitMode::WAIT);

    // управление вертикальным каналом на скорости с помощью дифферента
    // * value -- значение вертикального канала
    // * mode -- режим выбора вертикального канала
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_vert(float value, SpeedyVertMode mode, float timeout, WaitMode wm = WaitMode::WAIT);
};
