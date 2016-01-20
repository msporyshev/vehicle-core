#pragma once

#include "regulator.h"

#include <motion/CmdFixVert.h>
#include <speedy_vert_mode.h>

#include "controllers/pid_controller.h"

struct SpeedyVertRegulConfig
{
    SpeedyVertRegulConfig(const YamlReader& config);
    ~SpeedyVertRegulConfig();

    double min_height;
    double max_delta_depth;

    double kp;
    double ki;
    double kd;
    double accuracy;
};

class SpeedyVertRegulator : public Regulator
{
public:
    SpeedyVertRegulator(motion::CmdFixVert msg, std::shared_ptr<const SpeedyVertRegulConfig> config);
    ~SpeedyVertRegulator();
protected:
    void update(const NavigInfo& msg) override;
private:
    PIDController controller;
    SpeedyVertMode mode;
    double target_value;
    std::shared_ptr<const SpeedyVertRegulConfig> config;
};