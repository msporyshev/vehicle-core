#include "motion_client.h"

class RobosubMotionClient : public MotionClient
{
public:

    RobosubMotionClient(ipc::Communicator& com);
    ~RobosubMotionClient();

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

    // корректировка положения аппарата в горизонтальной плоскости
    // * thrust_forward -- доля от максимальной тяги в продольной оси: [-1; 1]
    // * thrust_right -- доля от максимальной тяги в поперечной оси: [-1; 1]
    // * delta_heading -- отклонение курса, положительной направления по часовой стрелке
    // * delta_depth -- изменение глубины, положительное направление вниз
    void adjust(double thrust_forward, double thrust_right, double delta_heading, double delta_depth);

    // отключение всех регуляторов
    void unfix_all();

    // включение стабилизации нулевого дифферента на время всей миссии
    void fix_pitch();

    // управление курсом
    // * value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_heading(double heading, WaitMode wm = WaitMode::WAIT);
    void fix_heading(double heading, double timeout, WaitMode wm = WaitMode::WAIT);

    // поворот по курсу на заданное количество радиан
    // * value -- значение в радианах, на которое требуется изменить курс
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_right(double bearing, WaitMode wm = WaitMode::WAIT);
    void turn_right(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, WaitMode wm = WaitMode::WAIT);
    void turn_left(double bearing, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление глубиной
    // * value -- значение глубины в метрах, положительное направление вниз, 0 соответствует поверхности
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_depth(double depth, WaitMode wm = WaitMode::WAIT);
    void fix_depth(double depth, double timeout, WaitMode wm = WaitMode::WAIT);

    // изменение глубины на заданное количество метров
    // * value -- значение в метрах, на которое требуется изменить глубину
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_down(double depth, WaitMode wm = WaitMode::WAIT);
    void move_down(double depth, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_up(double depth, WaitMode wm = WaitMode::WAIT);
    void move_up(double depth, double timeout, WaitMode wm = WaitMode::WAIT);

    // стабилизация положения в горизонтальной плоскости в глобальной системе координат
    // * value -- двумерная точка, описывающая положение.
    //   x -- вправо, в метрах
    //   y -- вперед, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_position(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    // сместиться в горизонтальной плоскости на заданные значения по продольной и поперечной оси
    // * value -- двумерная точка, описывающая положение.
    //   x -- вправо, в метрах
    //   y -- вперед, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void unseat(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    // движение строго по заданным направлениям на необходимое расстояние
    // выход к точке осуществляется в HOVER-режиме
    // * value -- расстояние, на которое нужно сместиться
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_forward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);

private:

    const double FIX_TIMEOUT = 1800.0;
    const double ADJUST_TIMEOUT = 5.0;
};
