#pragma once

#include "regulators/regulator.h"
#include "regul_storage.h"
#include "regul_producer.h"
#include "navig_info.h"

#include <navig/MsgNavigAngles.h>
#include <navig/MsgNavigDepth.h>
#include <navig/MsgNavigHeight.h>
#include <navig/MsgNavigPosition.h>
#include <navig/MsgNavigVelocity.h>

#include <motion/MsgRegul.h>
#include <libauv/config_reader/yaml_reader.h>

#include <vector>
#include <list>
#include <memory>

#include <libipc/ipc.h>
#include <ros/ros.h>

class MotionServer
{
public:
    static const std::string NODE_NAME;
    
    MotionServer(ipc::Communicator& communicator, const YamlReader& config);
    ~MotionServer();

    void handle_angles(const navig::MsgNavigAngles& msg);
    void handle_depth(const navig::MsgNavigDepth& msg);
    void handle_height(const navig::MsgNavigHeight& msg);
    void handle_position(const navig::MsgNavigPosition& msg);
    void handle_velocity(const navig::MsgNavigVelocity& msg);

    void init_ipc();
    
    // метод, запускающий сервер
    void run();

private:

    // ограничения тяги по соответствующим осям
    double tx_limit;
    double ty_limit;
    double tz_limit;
    double mx_limit;
    double my_limit;
    double mz_limit;

    NavigInfo navig;

    // ожидающие начала команды (еще не активированные)
    RegulStorage pending_list;

    // активные команды (активированные и выполняющиеся в данный момент)
    RegulStorage active_list;

    // составление конфигурационных параметров для регулятора
    YAML::Node make_regul_config(std::string name, YamlReader config,
        std::vector<std::string> rejected_dependencies = {}) const;

    ipc::Communicator& communicator_; ///> для подписки на сообщения
    ros::Publisher cmd_status_pub_, regul_pub_;

    // метод, в котором обрабатывается навигационное сообщение
    void process_navig(const NavigInfo& msg);

    // чтение конфигурации
    void read_config(YamlReader config);

    // обновление списка активных регуляторов
    // здесь же происходит отправка статусов команд на клиент
    void update_activity_list();

    // обновление тяг по всем осям и отправка на модуль управления ДРК
    void update_thrusts(const NavigInfo& msg);

    // публикация статуса команды
    void publish_cmd_status(int id, CmdStatus status);

    // ограниение величины заданными рамками
    double bound(double num, double limit);

    // конвертация системы координат в старую (для обратной совместимости)
    motion::MsgRegul convert(const motion::MsgRegul& msg) const;
};
