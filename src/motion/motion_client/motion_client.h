/**
\file
\brief Заголовочный файл клиента для регуляторов

В данном файле находятся обработку и публикацию сообщений, отправляемых 
регуляторам

*/

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

    MotionClient(ipc::Communicator& com);
    ~MotionClient();

protected:

    /**
    Прямое управление тягой по заданной оси
    \param[in] axis -- выбранная ось управления:
    \param[in] value -- доля от максимальной тяги: [-1; 1]
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void thrust(Axis axis, double value, double timeout, WaitMode wm = WaitMode::DONT_WAIT);

    /**
    Отключение всех регуляторов
    */
    void unfix_all();

    /**
    Управление курсом
    \param[in] value -- значение курса в радианах, положительное направление по часовой стрелке, 0 -- север
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] kp, ki, kd -- коэффициенты пид-регулятора
    \param[in] wm -- режим ожидания команды
    */
    void fix_heading(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_heading(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    /**
    Поворот по курсу на заданное количество радиан
    \param[in] value -- значение в радианах, на которое требуется изменить курс
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void turn_left(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_right(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление дифферентом
    \param[in] value -- значение дифферента в радианах, положительное направление соответствует повороту носа вверх, 0 -- горизонтальное положение
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] kp, ki, kd -- коэффициенты пид-регулятора
    \param[in] wm -- режим ожидания команды
    */
    void fix_pitch(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_pitch(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    /**
    Поворот по дифференту на заданное количество радиан
    \param[in] value -- значение в радианах, на которое требуется изменить дифферент
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void turn_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void turn_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Управление глубиной
    \param[in] value -- значение глубины в метрах, положительное направление вниз, 0 соответствует поверхности
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    \param[in] kp, ki, kd -- коэффициенты пид-регулятора
    */
    void fix_depth(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_depth(double value, double timeout, double kp, double ki, double kd, WaitMode wm = WaitMode::WAIT);

    /**
    Изменение глубины на заданное количество метров
    \param[in] value -- значение в метрах, на которое требуется изменить глубину
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void move_down(double value, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_up(double value, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Стабилизация положения в горизонтальной плоскости в глобальной системе координат
    \param[in] value -- двумерная точка, описывающая положение. x -- вперед, в метрах, y -- вправо, в метрах
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] fwd_kp, fwd_ki, fwd_kd -- коэффициенты пид-регулятора продольного движения
    \param[in] side_kp, side_ki, side_kd -- коэффициенты пид-регулятора поперечного движения
    \param[in] wm -- режим ожидания команды
    */
    void fix_position(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void fix_position(libauv::Point2d value, MoveMode move_mode, double timeout,
        double fwd_kp, double fwd_ki, double fwd_kd, double side_kp, double side_ki, double side_kd,
        WaitMode wm = WaitMode::WAIT);

    /**
    Сместиться в горизонтальной плоскости на заданные значения по продольной и поперечной оси
    \param[in] value -- двумерная точка, описывающая положение. x -- вперед, в метрах, y -- вправо, в метрах
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void unseat(libauv::Point2d value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

    /**
    Движение строго по заданным направлениям на необходимое расстояние
    \param[in] value -- двумерная точка расстояний по оси x и y:x -- вперед, в метрах, y -- вправо, в метрах
    \param[in] move_mode -- режим выхода к точке
    \param[in] timeout -- максимальное время работы регулятора в секундах
    \param[in] wm -- режим ожидания команды
    */
    void move_forward(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_backward(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_right(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);
    void move_left(double value, MoveMode move_mode, double timeout, WaitMode wm = WaitMode::WAIT);

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
private:
    ipc::Communicator& communicator_;

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
        ROS_INFO("Message %s published", typeid(CmdType).name());
        ROS_INFO("Id = %d, timeout = %f", cmd.id, cmd.timeout);
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
