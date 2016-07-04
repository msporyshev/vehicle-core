#pragma once

#include "regul_storage.h"
#include "regul_config.h"

#include <libipc/ipc.h>

#include <log.h>
#include <config_reader/yaml_reader.h>
#include <memory>

#include <log.h>

class BaseRegulProducer
{
public:
    virtual void init(std::shared_ptr<RegulStorage> regul_storage, const YamlReader& config,
        ipc::Communicator& com) = 0;
    std::shared_ptr<RegulConfig> base_config;
protected:
    std::shared_ptr<RegulStorage> storage;
};

template<typename RegulType, typename MsgType, typename ConfigType>
class RegulProducer : public BaseRegulProducer
{
public:

    virtual void init(std::shared_ptr<RegulStorage> regul_storage, const YamlReader& config,
        ipc::Communicator& com) override
    {
        storage = regul_storage;
        regul_config = std::make_shared<ConfigType>(config);
        base_config = regul_config;

        LOG << "subscribing " << ipc::classname(MsgType()) << std::endl;
        com.subscribe_cmd(&RegulProducer::handle_msg, this);
    }

    void handle_msg(const MsgType& msg)
    {
        LOG << "adding command to waiting_list -- #" << msg.id
            << " msg_type: " << ipc::classname(msg) << " timeout: " << msg.timeout << std::endl;
        storage->add(std::make_shared<RegulType>(msg, regul_config));
    }
private:
    std::shared_ptr<ConfigType> regul_config;
};
