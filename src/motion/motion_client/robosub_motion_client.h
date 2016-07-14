#pragma once

#include "motion_client.h"

class RobosubMotionClient : public MotionClient
{
public:

    RobosubMotionClient(ipc::Communicator& com);
    ~RobosubMotionClient();

    /**
    Прямое управление тягой по заданной оси
    \param[in] value -- доля от максимальной тяги: [-1; 1]
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void thrust_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void thrust_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Корректировка положения аппарата в горизонтальной плоскости
    \param[in] thrust_forward -- доля от максимальной тяги в продольной оси: [-1; 1]
    \param[in] thrust_right -- доля от максимальной тяги в поперечной оси: [-1; 1]
    \param[in] delta_heading -- отклонение курса, положительной направления по часовой стрелке
    \param[in] delta_depth -- изменение глубины, положительное направление вниз
    */
    void adjust(double thrust_forward, double thrust_right, double delta_heading, double delta_depth);

    /**
    Отключение всех регуляторов
    */
    void unfix_all();

    /**
    Включение стабилизации нулевого дифферента на время всей миссии
    */
    void fix_pitch();

    /**
    Управление курсом
    \param[in] value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_heading(double heading, WaitMode wm = WaitMode::WAIT);
    void fix_heading(double heading, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Поворот по курсу на заданное количество радиан
    \param[in] value -- значение в радианах, на которое требуется изменить курс
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void turn_right(double bearing, WaitMode wm = WaitMode::WAIT);
    void turn_right(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление глубиной
    \param[in] value -- значение глубины в метрах, положительное направление вниз, 0 соответствует поверхности
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_depth(double depth, WaitMode wm = WaitMode::WAIT);
    void fix_depth(double depth, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Изменение глубины на заданное количество метров
    \param[in] value -- значение в метрах, на которое требуется изменить глубину
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void move_down(double depth, WaitMode wm = WaitMode::WAIT);
    void move_down(double depth, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_up(double depth, WaitMode wm = WaitMode::WAIT);
    void move_up(double depth, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Стабилизация положения в горизонтальной плоскости в глобальной системе координат
    \param[in] value -- двумерная точка, описывающая положение. x -- вправо, в метрах, y -- вперед, в метрах?
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void fix_position(Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    void fix_target(Point2d value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_target(Point2d value, double distance, double timeout, WaitMode wm = WaitMode::WAIT);
    /**
    Cместиться в горизонтальной плоскости на заданные значения по продольной и поперечной оси
    \param[in] value -- двумерная точка, описывающая положение. x -- вправо, в метрах, y -- вперед, в метрах,
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void unseat(Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Движение строго по заданным направлениям на необходимое расстояние
    выход к точке осуществляется в HOVER-режиме
    \param[in] value -- расстояние, на которое нужно сместиться
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void move_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);

private:

    const double FIX_TIMEOUT = 1800.0;
    const double ADJUST_TIMEOUT = 5.0;
};
