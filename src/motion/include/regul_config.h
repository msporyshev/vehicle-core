#pragma once

#include <motion/CmdReconfigure.h>
#include <config_reader/yaml_reader.h>

#include <string>

struct RegulConfig
{
    virtual void reconfigure(const motion::CmdReconfigure& msg) {}
};

struct PidRegulConfig: RegulConfig
{
    double kp;
    double ki;
    double kd;

    PidRegulConfig(const YamlReader& config)
    {
        config.SET_PARAM(kp);
        config.SET_PARAM(ki);
        config.SET_PARAM(kd);
    }

    void reconfigure(const motion::CmdReconfigure& msg) override
    {
        if (msg.name == "kp") {
            kp = msg.value;
        } else if (msg.name == "kd") {
            kd = msg.value;
        } else if (msg.name == "ki") {
            ki = msg.value;
        }
    }
};