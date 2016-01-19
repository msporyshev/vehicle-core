#pragma once

#include "regul_storage.h"

// #include <ipc_lib.h>
#include <log.h>
#include <config_reader/yaml_reader.h>
#include <memory>

#include <log.h>

class BaseRegulProducer
{
public:
    virtual void init(std::shared_ptr<RegulStorage> regul_storage, const YamlReader& config) = 0;
};

template<typename RegulType, typename MsgType, typename ConfigType>
class RegulProducer : public BaseRegulProducer
{
public:

    virtual void init(std::shared_ptr<RegulStorage> regul_storage, const YamlReader& config) override
    {
        storage = regul_storage;
        regul_config = std::make_shared<ConfigType>(config);
        // LOG << "subscribing " << MsgType::IPC_NAME << std::endl;
        // Central::subscribe(*this, handle_msg);
    }

    void handle_msg(MsgType msg)
    {
        LOG << "adding command to waiting_list -- #" << msg.id
            << " msg_type: " << MsgType::IPC_NAME << " timeout: " << msg.timeout << std::endl;
        storage->add(std::make_shared<RegulType>(msg, regul_config));
    }
private:
    std::shared_ptr<RegulStorage> storage;
    std::shared_ptr<ConfigType> regul_config;
};
