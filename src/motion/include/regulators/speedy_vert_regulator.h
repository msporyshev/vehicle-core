#pragma once

#include "regulator.h"
#include "regul_config.h"

#include <motion/CmdFixVert.h>
#include <speedy_vert_mode.h>

#include "controllers/pid_controller.h"

struct SpeedyVertRegulConfig: PidRegulConfig
{
    SpeedyVertRegulConfig(const YamlReader& config);

    double min_height;
    double max_delta_depth;

    double accuracy;
};

class SpeedyVertRegulator : public Regulator
{
public:
    SpeedyVertRegulator(motion::CmdFixVert msg, std::shared_ptr<const SpeedyVertRegulConfig> config);
protected:
    void update(const NavigInfo& msg) override;
private:
    PIDController controller;
    SpeedyVertMode mode;
    double target_value;
    std::shared_ptr<const SpeedyVertRegulConfig> config;
};
