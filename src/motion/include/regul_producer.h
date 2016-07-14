#pragma once

#include "regul_storage.h"
#include "regul_config.h"

#include <motion/CmdReconfigure.h>

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

    virtual void reconfigure(const motion::CmdReconfigure& msg) = 0;
protected:
    std::shared_ptr<RegulStorage> storage;
};

template<typename RegulType, typename CmdType, typename ConfigType>
class RegulProducer : public BaseRegulProducer
{
public:

    void init(std::shared_ptr<RegulStorage> regul_storage, const YamlReader& config,
        ipc::Communicator& com) override
    {
        storage = regul_storage;
        regul_config = std::make_shared<ConfigType>(config);

        LOG << "subscribing " << ipc::classname(CmdType()) << std::endl;
        com.subscribe_cmd(&RegulProducer::handle_msg, this);
    }

    void reconfigure(const motion::CmdReconfigure& msg) override
    {
        regul_config->reconfigure(msg);
        // *last_regul = RegulType(last_cmd, regul_config);
        handle_msg(last_cmd);
    }

    void handle_msg(const CmdType& msg)
    {
        LOG << "adding command to waiting_list -- #" << msg.id
            << " msg_type: " << ipc::classname(msg) << " timeout: " << msg.timeout << std::endl;
        last_regul = std::make_shared<RegulType>(msg, regul_config);
        last_cmd = msg;
        storage->add(last_regul);
    }
private:
    std::shared_ptr<ConfigType> regul_config;
    std::shared_ptr<RegulType> last_regul;
    CmdType last_cmd;
};
