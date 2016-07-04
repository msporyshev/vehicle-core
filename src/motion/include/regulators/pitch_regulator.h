#pragma once

#include "regulator.h"
#include "regul_config.h"

#include "controllers/pid_controller.h"

#include <motion/CmdFixPitch.h>
#include <motion/CmdFixPitchConf.h>
#include <coord_system.h>

struct PitchRegulConfig: PidRegulConfig
{
    PitchRegulConfig(const YamlReader& config);

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;
};

class PitchRegulator : public Regulator
{
public:
    PitchRegulator(motion::CmdFixPitch msg, std::shared_ptr<const PitchRegulConfig> config);
    PitchRegulator(motion::CmdFixPitchConf msg, std::shared_ptr<const PitchRegulConfig> config);
    ~PitchRegulator();

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;
    double cmd_pitch;
    double target_pitch;
    CoordSystem coord_system;

    std::shared_ptr<const PitchRegulConfig> config;
};
