/**
\file
\brief Навигационный модуль расчета навигационных данных аппарата

Модуль содержит подписку на сообщения сенсорных устройств
и основной цикл навига, в котором публикуются навигационные данные
*/

// Общие библиотеки и служебные функции навига
#include <utils/node_utils.h>
#include <utils/utils.h>
#include <libipc/ipc.h>
#include <iostream>
#include "navig.h"

// Принимаемые сообщения
#include <supervisor/MsgDepth.h>        ///< Глубина и скорость изменения глубины от супервизора
#include <dvl/MsgDown.h>                ///< Дальность вниз до грунта и скорость сближения с грунтом от доплера
#include <dvl/MsgPlaneVelocity.h>       ///< Вектор скорости (вперед, вправо) по доплеру
#include <compass/MsgAngle.h>           ///< Курс, дифферент, крен от компаса
#include <compass/MsgAngleRate.h>       ///< Скорость курса, дифферента и крена от компаса
#include <compass/MsgAcceleration.h>    ///< Ускорения вперед, вправо и вниз от компаса
#include <gps/MsgGlobalPosition.h>      ///< Глобальные координаты от GPS

// Публикуемые сообщения
#include <navig/MsgDepth.h>             ///< Глубина и скорость изменения глубины
#include <navig/MsgHeight.h>            ///< Высота над грунтом и скорость сближения с грунтом
#include <navig/MsgPlaneVelocity.h>     ///< Вектор скорости в горизонтальной плоскости аппарата (вперед, вправо)
#include <navig/MsgAngle.h>             ///< Курс, дифферент, крен
#include <navig/MsgAngleRate.h>         ///< Скорость курса, дифферента и крена
#include <navig/MsgLocalPosition.h>     ///< Локальные координаты аппарата (север, восток) в метрах
#include <navig/MsgGlobalPosition.h>    ///< Глобальные координаты аппарата (широта, долгота) в градусах
#include <navig/MsgRaw.h>               ///< Сырые отладочные данные навига
#include <navig/MsgOdometry.h>          ///< Публикуемое обобщенное сообщение телеметрии с флагами

using namespace this_node;
using namespace this_node::ipc;

int main(int argc, char* argv[])
{
    this_node::init(argc, argv);

    // Считываем настройки нода из конфига и параметров запуска
    const double RATE        = config::read_as<double>("rate");     // Частота работы модуля [Гц]
    const double AGE_MAX     = config::read_as<double>("age_max");  // Максимальный возраст сообщений от поставщиков данных [c]
    const double START_DELAY = config::read_as<double>("start_delay");  // Время ожидания нода перед стартом основного цикла [c]

    // Подписываемся на прием сообщений
    auto sub_supervisor_depth     = subscribe<supervisor::MsgDepth>("supervisor");
    auto sub_dvl_down             = subscribe<dvl::MsgDown>("dvl");
    auto sub_dvl_plane_velocity   = subscribe<dvl::MsgPlaneVelocity>("dvl");
    auto sub_compass_angle        = subscribe<compass::MsgAngle>("compass");
    auto sub_compass_angle_rate   = subscribe<compass::MsgAngleRate>("compass");
    auto sub_compass_acceleration = subscribe<compass::MsgAcceleration>("compass");
    auto sub_gps_global_position  = subscribe<gps::MsgGlobalPosition>("gps");

    // Подписываемся на публикацию сообщений
    auto pub_navig_depth           = advertise<navig::MsgDepth>();
    auto pub_navig_height          = advertise<navig::MsgHeight>();
    auto pub_navig_plane_velocity  = advertise<navig::MsgPlaneVelocity>();
    auto pub_navig_angle           = advertise<navig::MsgAngle>();
    auto pub_navig_angle_rate      = advertise<navig::MsgAngleRate>();
    auto pub_navig_local_position  = advertise<navig::MsgLocalPosition>();
    auto pub_navig_global_position = advertise<navig::MsgGlobalPosition>();
    auto pub_navig_odometry        = advertise<navig::MsgOdometry>();
    auto pub_navig_raw             = advertise<navig::MsgRaw>();

    // Заводим переменные публикуемых данных
    navig::MsgDepth          navig_depth;
    navig::MsgHeight         navig_height;
    navig::MsgPlaneVelocity  navig_plane_velocity;
    navig::MsgAngle          navig_angle;
    navig::MsgAngleRate      navig_angle_rate;
    navig::MsgLocalPosition  navig_local_position;
    navig::MsgGlobalPosition navig_global_position;
    navig::MsgOdometry       odometry;
    navig::MsgRaw            raw;

    // Ожидаем пока включатся и начнут публиковать остальные ноды
    ros::Duration(START_DELAY).sleep();

    // Запускаем цикл обмена сообщениями
    ::ipc::EventLoop loop(RATE);
    while(loop.ok()) {

        // Определяем временную задержку с предыдущего цикла
        double time_now = timestamp();
        static double time_was = time_now;
        double dt = time_now - time_was;
        time_was = time_now;

        // Считываем исходные данные
        auto supervisor_depth     = sub_supervisor_depth    .msg();
        auto dvl_down             = sub_dvl_down            .msg();
        auto dvl_plane_velocity   = sub_dvl_plane_velocity  .msg();
        auto compass_angle        = sub_compass_angle       .msg();
        auto compass_angle_rate   = sub_compass_angle_rate  .msg();
        auto compass_acceleration = sub_compass_acceleration.msg();
        auto gps_global_position  = sub_gps_global_position .msg();

        // Заполняем служебные поля в отладочном сообщении
        raw.real_period = dt;
        update_age_info(raw.supervisor_depth    , sub_supervisor_depth    .age(), AGE_MAX);
        update_age_info(raw.dvl_down            , sub_dvl_down            .age(), AGE_MAX);
        update_age_info(raw.dvl_plane_velocity  , sub_dvl_plane_velocity  .age(), AGE_MAX);
        update_age_info(raw.compass_angle       , sub_compass_angle       .age(), AGE_MAX);
        update_age_info(raw.compass_angle_rate  , sub_compass_angle_rate  .age(), AGE_MAX);
        update_age_info(raw.compass_acceleration, sub_compass_acceleration.age(), AGE_MAX);
        update_age_info(raw.gps_global_position , sub_gps_global_position .age(), AGE_MAX);

        // Интегрирование вектора СКОРОСТИ ПО УСКОРЕНИЯМ (только если приходят ускорения)
        if(raw.compass_acceleration.fresh) {
            integrate_plane(raw.velocity_acc, compass_acceleration, dt);
        }

        // ОБОБЩЕННАЯ СКОРОСТЬ всегда берется от доплера
        raw.velocity_flag    = raw.dvl_plane_velocity.fresh;
        raw.velocity.right   = dvl_plane_velocity.right;
        raw.velocity.forward = dvl_plane_velocity.forward;

        // ВСЕГДА интегрируем служебные ТРАЕКТОРИИ из разных скоростей, даже не смотря на возраст компаса
        integrate_local(raw.position_dvl, dvl_plane_velocity, compass_angle.heading, dt);
        integrate_local(raw.position_acc, raw.velocity_acc  , compass_angle.heading, dt);

        // Интегрируем ОБОБЩЕННУЮ ТРАЕКТОРИЮ только, если если есть обобщенная скорость и курс
        raw.position_flag = (raw.velocity_flag && raw.compass_angle.fresh);
        if(raw.position_flag) {
            integrate_local(raw.position, raw.velocity, compass_angle.heading, dt);
        }

        // ЗАПОЛНЯЕМ ВСЕ СООБЩЕНИЯ, кроме уже заполненного сырого
        navig_depth.distance            = supervisor_depth.distance;
        navig_depth.velocity            = supervisor_depth.velocity;
        navig_height.distance           = dvl_down.distance;           /// \todo Сделать пересчет с учетом крена/дифферента/ЭЛС
        navig_height.velocity           = dvl_down.velocity;           /// \todo Сделать пересчет с учетом крена/дифферента/ЭЛС
        navig_angle.heading             = compass_angle.heading;
        navig_angle.pitch               = compass_angle.pitch;
        navig_angle.roll                = compass_angle.roll;
        navig_angle_rate.heading        = compass_angle_rate.heading;
        navig_angle_rate.pitch          = compass_angle_rate.pitch;
        navig_angle_rate.roll           = compass_angle_rate.roll;
        navig_plane_velocity.right      = raw.velocity.right;
        navig_plane_velocity.forward    = raw.velocity.forward;
        navig_local_position.east       = raw.position.east;
        navig_local_position.north      = raw.position.north;
        navig_global_position.latitude  = gps_global_position.latitude;
        navig_global_position.longitude = gps_global_position.longitude;

        // Всегда публикуем сырые данные, а основные - только если они свежие
        pub_navig_raw.publish(raw);
        if(raw.supervisor_depth   .fresh) pub_navig_depth          .publish(navig_depth);
        if(raw.dvl_down           .fresh) pub_navig_height         .publish(navig_height);
        if(raw.compass_angle      .fresh) pub_navig_angle          .publish(navig_angle);
        if(raw.compass_angle_rate .fresh) pub_navig_angle_rate     .publish(navig_angle_rate);
        if(raw.velocity_flag            ) pub_navig_plane_velocity .publish(navig_plane_velocity);
        if(raw.position_flag            ) pub_navig_local_position .publish(navig_local_position);
        if(raw.gps_global_position.fresh) pub_navig_global_position.publish(navig_global_position);

        // Публикуем обобщенное сообщение телеметрии с флагами
        odometry.has_angle    = raw.compass_angle.fresh;
        odometry.angle        = navig_angle;
        odometry.has_depth    = raw.supervisor_depth.fresh;
        odometry.depth        = navig_depth;
        odometry.has_height   = raw.dvl_down.fresh;
        odometry.height       = navig_height;
        odometry.has_velocity = raw.velocity_flag;
        odometry.velocity     = navig_plane_velocity;
        odometry.has_pos      = raw.position_flag;
        odometry.pos          = navig_local_position;
        pub_navig_odometry.publish(odometry);
    }

    return 0;
}
