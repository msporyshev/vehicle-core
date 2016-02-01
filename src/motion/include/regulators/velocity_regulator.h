#pragma once

#include "regulator.h"

#include "controllers/pid_controller.h"

#include <motion/CmdFixVelocity.h>

struct VelocityRegulConfig
{
    VelocityRegulConfig(const YamlReader& config);
    ~VelocityRegulConfig();

    double kp;
    double ki;
    double kd;

    double accuracy;
};

class VelocityRegulator : public Regulator
{
public:
    VelocityRegulator(motion::CmdFixVelocity msg, std::shared_ptr<const VelocityRegulConfig> config);
    ~VelocityRegulator();

protected:
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;
    double target_velocity;

    std::shared_ptr<const VelocityRegulConfig> config;
};
