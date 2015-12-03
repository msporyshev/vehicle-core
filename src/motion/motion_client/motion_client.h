#include <array>
#include <map>

#include "move_mode.h"
#include "cmd_status.h"
#include "wait_mode.h"
#include "axis.h"
#include "coord_system.h"
#include "speedy_vert_mode.h"

#include <libauv/point/point.h>
#include <motion/MsgCmdStatus.h>
#include <libauv/include/axes.h>

#include <libipc/ipc.h>

#include <typeinfo>

class MotionClient
{
public:

    MotionClient(ipc::Communicator* com);
    ~MotionClient();

protected:

    // прямое управление тягой по заданной оси
    // * axis -- выбранная ось управления:
    // * value -- доля от максимальной тяги: [-1; 1]
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void thrust(Axis axis, double value, double timeout, WaitMode wm = WaitMode::DONT_WAIT);

    // отключение всех регуляторов
    void unfix_all();

    // управление курсом
    // * value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    // * timeout -- максимальное время работы регулятора в секундах
    // * kp, ki, kd -- коэффициенты пид-регулятора
    // * wm -- режим ожидания команды
    void fix_heading(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_heading(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    // поворот по курсу на заданное количество радиан
    // * value -- значение в радианах, на которое требуется изменить курс
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление дифферентом
    // * value -- значение дифферента в радианах, положительное направление соответствует повороту носа вверх,
    //   0 -- горизонтальное положение
    // * timeout -- максимальное время работы регулятора в секундах
    // * kp, ki, kd -- коэффициенты пид-регулятора
    // * wm -- режим ожидания команды
    void fix_pitch(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_pitch(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    // поворот по дифференту на заданное количество радиан
    // * value -- значение в радианах, на которое требуется изменить дифферент
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void turn_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление глубиной
    // * value -- значение глубины в метрах, положительное направление вниз, 0 соответствует поверхности
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    // * kp, ki, kd -- коэффициенты пид-регулятора
    void fix_depth(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_depth(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    // изменение глубины на заданное количество метров
    // * value -- значение в метрах, на которое требуется изменить глубину
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // стабилизация положения в горизонтальной плоскости в глобальной системе координат
    // * value -- двумерная точка, описывающая положение.
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * fwd_kp, fwd_ki, fwd_kd -- коэффициенты пид-регулятора продольного движения
    // * side_kp, side_ki, side_kd -- коэффициенты пид-регулятора поперечного движения
    // * wm -- режим ожидания команды
    void fix_position(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_position(libauv::Point2d value, MoveMode move_mode, double timeout,
        double fwd_kp, double fwd_ki, double fwd_kd, double side_kp, double side_ki, double side_kd,
        WaitMode wm = WaitMode::WAIT);

    // сместиться в горизонтальной плоскости на заданные значения по продольной и поперечной оси
    // * value -- двумерная точка, описывающая положение.
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void unseat(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    // движение строго по заданным направлениям на необходимое расстояние
    // * value -- двумерная точка расстояний по оси x и y:
    //   x -- вперед, в метрах
    //   y -- вправо, в метрах
    // * move_mode -- режим выхода к точке
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void move_forward(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление продольной скоростью
    // * value -- значение скорости, в метрах в секунду
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_velocity(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    // управление вертикальным каналом на скорости с помощью дифферента
    // * value -- значение вертикального канала
    // * mode -- режим выбора вертикального канала
    // * timeout -- максимальное время работы регулятора в секундах
    // * wm -- режим ожидания команды
    void fix_vert(double value, SpeedyVertMode mode, double timeout, WaitMode wm = WaitMode::WAIT);
private:
    std::shared_ptr<ipc::Communicator> communicator_;

    std::map<std::string, ros::Publisher> publishers_;
    std::map<int, CmdStatus> cmd_history;
    int last_cmd_id = 0;

    int generate_cmd_id();
    void handle_msg_cmd_status(const motion::MsgCmdStatus& msg);
    CmdStatus wait_for(int id);

    template<typename CmdType>
    void publish_cmd(CmdType cmd, double timeout, WaitMode wm)
    {
        cmd.id = generate_cmd_id();
        cmd.timeout = timeout;
        publishers_[typeid(CmdType).name()].publish(cmd);
        if (wm == WaitMode::WAIT) {
            wait_for(cmd.id);
        }
    }

    void fix_heading(double value, CoordSystem coord_system, double timeout, WaitMode wm);
    void fix_pitch(double value, CoordSystem coord_system, double timeout, WaitMode wm);
    void fix_depth(double value, CoordSystem coord_system, double timeout, WaitMode wm);

    void fix_position(libauv::Point2d value, MoveMode move_mode, CoordSystem coord_system, double timeout,
        WaitMode wm);
};
