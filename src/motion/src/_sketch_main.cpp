/* Набросок нового простого регулятора */

// Общие библиотеки и служебные функции
#include <utils/node_utils.h>
#include <utils/utils.h>
#include <libipc/ipc.h>
#include <iostream>
#include "motion.h"

// Принимаемые сообщения
#include <navig/MsgDepth.h>             // Глубина и скорость изменения глубины
#include <navig/MsgHeight.h>            // Высота над грунтом и скорость сближения с грунтом
#include <navig/MsgPlaneVelocity.h>     // Вектор скорости в горизонтальной плоскости аппарата (вперед, вправо)
#include <navig/MsgAngle.h>             // Курс, дифферент, крен
#include <navig/MsgAngleRate.h>         // Скорость курса, дифферента и крена
#include <navig/MsgLocalPosition.h>     // Локальные координаты аппарата (север, восток) в метрах
#include <motion/CmdPitch.h>            // Управление по дифференту
#include <motion/CmdDown.h>             // Управление по вертикальному каналу
#include <motion/CmdTack.h>             // Движение галсом (управление по курсу и скоростям)
#include <motion/CmdLocalPosition.h>    // Движение в точку, заданную локальными координатами
#include <motion/CmdSettings.h>         // Настроечные параметры регулятора

// Публикуемые сообщения
#include <motion/MsgStatus.h>           // Статус выполнения команды
#include <motion/MsgRaw.h>              // Сырые отладочные данные регулятора
#include <tcu/CmdForce.h>               // Шестимерный вектор "упоров" БУДам [у.е.] в диапазоне [-1, +1]


using namespace this_node;
using namespace this_node::ipc;

int main(int argc, char *argv[])
{
    this_node::init(argc, argv);

    // Подписываемся на прием навигации, команд и настроек
    auto sub_depth              = subscribe<navig::MsgDepth>();
    auto sub_height             = subscribe<navig::MsgHeight>();
    auto sub_plane_velocity     = subscribe<navig::MsgPlaneVelocity>();
    auto sub_angle              = subscribe<navig::MsgAngle>();
    auto sub_angle_rate         = subscribe<navig::MsgAngleRate>();
    auto sub_local_position     = subscribe<navig::MsgLocalPosition>();
    auto sub_cmd_pitch          = subscribe<motion::CmdPitch>();
    auto sub_cmd_down           = subscribe<motion::CmdDown>();
    auto sub_cmd_tack           = subscribe<motion::CmdTack>();
    auto sub_cmd_local_position = subscribe<motion::CmdLocalPosition>();
    auto sub_settings           = subscribe<motion::CmdSettings>();

    // Считываем настройки
    motion::CmdSettings settings;                           // Сразу создаем переменную настроек (чтобы не затереть в главном цикле)
    fill_settings(settigs);                                 // и кладем в нее данные их конфига
    const double RATE = config::read_as<double>("rate");    // Считываем частоту работы модуля [Гц]

    // Заводим переменные публикуемых данных
    tcu::CmdForce     force;   auto pub_force  = advertise<tcu::CmdForce>();
    motion::MsgStatus status;  auto pub_status = advertise<motion::MsgStatus>();
    motion::MsgRaw    raw;     auto pub_raw    = advertise<motion::MsgRaw>();

    // Запускаем цикл обмена сообщениями
    ::ipc::EventLoop loop(RATE);
    while(loop.ok()) {

        // Считываем исходные навигационные данные, настройки и команды
        auto depth              = sub_depth             .msg();
        auto height             = sub_height            .msg();
        auto plane_velocity     = sub_plane_velocity    .msg();
        auto angle              = sub_angle             .msg();
        auto angle_rate         = sub_angle_rate        .msg();
        auto local_position     = sub_local_position    .msg();
        auto cmd_pitch          = sub_cmd_pitch         .msg();
        auto cmd_down           = sub_cmd_down          .msg();
        auto cmd_tack           = sub_cmd_tack          .msg();
        auto cmd_local_position = sub_cmd_local_position.msg();

        // Настройки только считываем, переменную заводить нельзя,
        settings = sub_settings.msg();  // т.к. в настройках уже есть параметры по умолчанию

        // Заполняем параметры по каналу "дифферента"
        raw.pitch.regulated = (sub_cmd_pitch.age() < cmd_pitch.timeout);
        if(raw.pitch.regulated) {
            raw.pitch.prop = cmd_pitch.value - angle.pitch;
            raw.pitch.diff = angle_rate.pitch;
            force.pitch = regulator_pd(raw.pitch, settings.pitch);
        }

        // Заполняем параметры по каналу "вниз"
        raw.down.regulated = (sub_cmd_down .age() < cmd_down.timeout);
        if(raw.pitch.regulated) {
            if(cmd_down.mode == HEIGHT) {   // Режим высоты
                raw.down.prop = height.distance - cmd_down.value;
                raw.down.diff = -height.velocity;
            } else {                        // Режим глубины
                raw.down.prop = cmd_down.value - depth.distance;
                raw.down.diff = depth.velocity;
            }
            force.down = regulator_pd(raw.down, settings.down);
        }

        // Заполняем параметры "в плоскости" в зависимости от режима
        raw.plane.mode = (sub_cmd_tack.age() < sub_cmd_local_position.age() ? MODE_TACK : MODE_LOCAL_POSITION);
        // В зависимости от режима в горизонтальной плоскости:
        switch (raw.plane.mode) {
        case MODE_TACK:                 // Движение галсом
            raw.plane.regulated  = (sub_cmd_tack.age() < cmd_tack.timeout);
            if(raw.plane.regulated) {   // Управление только в дальней зоне управляемся
                raw.velocity.forward = cmd_tack.forward;
                raw.velocity.right   = cmd_tack.right;
                raw.heading.prop     = cmd_tack.heading - angle.heading;
                raw.heading.diff     = angle_rate.heading;
            }
        break;
        case MODE_LOCAL_POSITION:       // Движение в точку
            raw.plane.distance  = distance(cmd_local_position.coord, local_position);
            raw.plane.regulated = (sub_cmd_local_position.age() < cmd_local_position.timeout)
                                   && (raw.plane.distance > settings.near_zone);
            if(raw.plane.regulated) {   // Управление только в дальней зоне управляемся
                raw.velocity.forward = cmd_local_position.forward;
                raw.velocity.right   = cmd_local_position.right;
                raw.heading.prop     = heading_to_point(cmd_local_position.coord, local_position) - angle.heading;
                raw.heading.diff     = angle_rate.heading;
            }
        break;
        }
        // Заполняем вектор упоров в плоскости
        if(raw.plane.regulated) {
            force.heading = regulator_pd(raw.heading, settings.heading);
            force.forward = raw.forward;
            force.right   = raw.right;
        }

        // Всегда публикуем сырое сообщение
        pub_raw.publish(raw);
        // Публикуем команду БУДам, если хоть что-нить регулируется
        //! TODO: Надо б сделать одну публикацию нулевых данных перед остановкой публикации
        if (raw.pitch.regulated || raw.down.regulated || raw.plane.regulated) {
            pub_force.publish(force);
        }
    }

    return 0;
}
