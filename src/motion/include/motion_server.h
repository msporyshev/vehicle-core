#pragma once

#include "regulators/regulator.h"
#include "regul_storage.h"
#include "regul_producer.h"
#include "navig_info.h"

#include <navig/MsgAngles.h>
#include <navig/MsgRates.h>
#include <navig/MsgDepth.h>
#include <navig/MsgHeight.h>
#include <navig/MsgPosition.h>
#include <navig/MsgVelocity.h>

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

    void handle_angles(const navig::MsgAngles& msg);
    void handle_rate(const navig::MsgRates& msg);
    void handle_depth(const navig::MsgDepth& msg);
    void handle_height(const navig::MsgHeight& msg);
    void handle_position(const navig::MsgPosition& msg);
    void handle_velocity(const navig::MsgVelocity& msg);

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

    int freq_; ///> Период с которым выполняется чтение сообщений [Гц]
    double timeout_silence_; ///> Таймаут, после которого считается, что устройсто "молчит" [с]
    double timeout_not_respond_; ///> Таймаут, после которого считается, что устройство больше ничего не присылает [c]

    // ожидающие начала команды (еще не активированные)
    RegulStorage pending_list;

    // активные команды (активированные и выполняющиеся в данный момент)
    RegulStorage active_list;

    // составление конфигурационных параметров для регулятора
    YAML::Node make_regul_config(std::string name, YamlReader config,
        std::vector<std::string> rejected_dependencies = {}) const;

    ipc::Communicator& communicator_; ///> для подписки на сообщения
    ipc::Subscriber<navig::MsgAngles> angles_msg_;
    ipc::Subscriber<navig::MsgRates> rates_msg_;
    ipc::Subscriber<navig::MsgDepth> depth_msg_;
    ipc::Subscriber<navig::MsgHeight> height_msg_;
    ipc::Subscriber<navig::MsgPosition> position_msg_;
    ipc::Subscriber<navig::MsgVelocity> velocity_msg_;
    ros::Publisher cmd_status_pub_, regul_pub_;

    ///>Время старта нода. Нужно, чтобы понимать, что сообщение нам вообще не приходит
    ros::Time start_time_ = ros::Time::now();

    /**
    Метод для чтения сообщений типа Msg в синхронном режиме и их обработка соответствующим хэндлером
    */
    template<typename Msg>
    void read_msg(ipc::Subscriber<Msg>& sub, void (MotionServer::*handle_msg)(const Msg&))
    {
        if (!sub.ready() && (ros::Time::now() - start_time_).toSec() > timeout_silence_) {
            // ROS_INFO_STREAM("Message " << ipc::classname(sub.msg()) << " hasn't been receiving for " << timeout_silence_ << " seconds.");
        } else if (ipc::is_actual(sub.msg(), timeout_not_respond_)) {
            (this->*handle_msg)(sub.msg());
        } else if (ipc::timestamp(sub.msg()) != 0.0) {
            // ROS_INFO_STREAM("Message " << ipc::classname(sub.msg()) << " from navig hasn't been receiving for " << timeout_not_respond_ << " seconds.");
        }
    }
 
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
