#pragma once

#include "regul_producer.h"

#include <map>

typedef std::map<std::string, std::vector<std::shared_ptr<BaseRegulProducer>>> Producers;
typedef std::vector<std::string> MsgNames;


class Registry
{
public:
    static void add(std::string name, std::string msg_name, std::shared_ptr<BaseRegulProducer> producer);
    static std::vector<std::shared_ptr<BaseRegulProducer>> get(std::string name);
private:
    static Producers& get_producers();
    static MsgNames& get_used_msgs();
    static std::shared_ptr<Producers> data;
    static std::shared_ptr<MsgNames> used_msgs;
};

template<typename RegulProducerType>
class Registrator
{
public:
    Registrator(std::string name, std::string msg_name)
    {
        Registry::add(name, msg_name, std::make_shared<RegulProducerType>());
    }
};

// макрос, регистрирующий регулятор
// NAME -- имя регулятора
// REGUL_TYPE -- тип регулятора
// MSG_TYPE -- тип сообщения, активирующего регулятор
// CONFIG_TYPE -- тип структуры, из которой считывается конфигурация при создании регулятора
// можно несколько раз регистрировать регуляторы
// уникальность должна поддерживаться на уровне пары NAME + MSG_TYPE
#define REG_REGUL(NAME, REGUL_TYPE, MSG_TYPE, CONFIG_TYPE) \
static Registrator<RegulProducer<REGUL_TYPE, MSG_TYPE, CONFIG_TYPE>> NAME##__##MSG_TYPE##__(#NAME, #MSG_TYPE)
