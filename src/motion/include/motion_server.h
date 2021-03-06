#pragma once

#include "regulators/regulator.h"
#include "regul_storage.h"
#include "regul_producer.h"
#include "navig_info.h"

#include <navig/MsgAngle.h>
#include <navig/MsgAngleRate.h>
#include <navig/MsgDepth.h>
#include <navig/MsgHeight.h>
#include <navig/MsgLocalPosition.h>
#include <navig/MsgGlobalPosition.h>
#include <navig/MsgPlaneVelocity.h>

#include <motion/CmdReconfigure.h>

#include <tcu/CmdForce.h>
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

    void handle_global_pos(const navig::MsgGlobalPosition& msg);
    void handle_angles(const navig::MsgAngle& msg);
    void handle_rate(const navig::MsgAngleRate& msg);
    void handle_depth(const navig::MsgDepth& msg);
    void handle_height(const navig::MsgHeight& msg);
    void handle_position(const navig::MsgLocalPosition& msg);
    void handle_velocity(const navig::MsgPlaneVelocity& msg);
    void handle_reconfigure(const motion::CmdReconfigure& msg);

    void init_ipc();

    // метод, запускающий сервер
    void run();

private:

    // ограничения тяги по соответствующим осям
    double forward_limit;
    double right_limit;
    double down_limit;
    double mforward_limit;
    double mright_limit;
    double mdown_limit;

    NavigInfo navig;

    int freq_; ///> Период с которым выполняется чтение сообщений [Гц]
    double timeout_silence_; ///> Таймаут, после которого считается, что устройсто "молчит" [с]
    double timeout_not_respond_; ///> Таймаут, после которого считается, что устройство больше ничего не присылает [c]

    // ожидающие начала команды (еще не активированные)
    std::shared_ptr<RegulStorage> pending_list;

    // активные команды (активированные и выполняющиеся в данный момент)
    RegulStorage active_list;

    // составление конфигурационных параметров для регулятора
    YAML::Node make_regul_config(std::string name, YamlReader config,
        std::vector<std::string> rejected_dependencies = {}) const;

    ipc::Communicator& communicator_; ///> для подписки на сообщения
    ipc::Subscriber<navig::MsgAngle> angles_msg_;
    ipc::Subscriber<navig::MsgAngleRate> rates_msg_;
    ipc::Subscriber<navig::MsgDepth> depth_msg_;
    ipc::Subscriber<navig::MsgHeight> height_msg_;
    ipc::Subscriber<navig::MsgLocalPosition> position_msg_;
    ipc::Subscriber<navig::MsgGlobalPosition> global_pos_msg_;
    ipc::Subscriber<navig::MsgPlaneVelocity> velocity_msg_;
    ipc::Subscriber<motion::CmdReconfigure> reconfigure_sub_;
    ros::Publisher cmd_status_pub_, regul_pub_;

    ///>Время старта нода. Нужно, чтобы понимать, что сообщение нам вообще не приходит
    ros::Time start_time_ = ros::Time::now();

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
};
