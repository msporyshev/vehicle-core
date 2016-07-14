#pragma once

#include "regulator.h"

#include "controllers/pid_controller.h"

#include "regul_config.h"
#include <motion/CmdFixHeight.h>
#include <coord_system.h>

struct HeightRegulConfig: PidRegulConfig
{
    HeightRegulConfig(const YamlReader& config);

    double buoyancy_thrust;

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;
};

class HeightRegulator : public Regulator
{
public:
    HeightRegulator(motion::CmdFixHeight msg, std::shared_ptr<const HeightRegulConfig> config);

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;
    double cmd_height;
    double target_height;
    CoordSystem coord_system;

    std::shared_ptr<const HeightRegulConfig> config;
};
