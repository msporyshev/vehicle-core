#pragma once

#include "regulator.h"

#include "controllers/pid_controller.h"

#include <motion/CmdFixHeading.h>
#include <motion/CmdFixHeadingConf.h>
#include <coord_system.h>

struct HeadingRegulConfig
{
    HeadingRegulConfig(const YamlReader& config);
    ~HeadingRegulConfig();

    double kp;
    double ki;
    double kd;

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;
};

class HeadingRegulator : public Regulator
{
public:
    HeadingRegulator(motion::CmdFixHeading msg, std::shared_ptr<const HeadingRegulConfig> config);
    HeadingRegulator(motion::CmdFixHeadingConf msg, std::shared_ptr<const HeadingRegulConfig> config);
    ~HeadingRegulator();

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;
    double cmd_heading;
    double target_heading;
    CoordSystem coord_system;

    std::shared_ptr<const HeadingRegulConfig> config;
};
