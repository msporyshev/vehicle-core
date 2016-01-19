#pragma once

#include "regulator.h"

#include "controllers/pid_controller.h"

#include <motion/CmdFixDepth.h>
#include <motion/CmdFixDepthConf.h>
#include <coord_system.h>

struct DepthRegulConfig
{
    DepthRegulConfig(const YamlReader& config);
    ~DepthRegulConfig();

    double kp;
    double ki;
    double kd;
    double buoyancy_thrust;

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;
};

class DepthRegulator : public Regulator
{
public:
    DepthRegulator(motion::CmdFixDepth msg, std::shared_ptr<const DepthRegulConfig> config);
    DepthRegulator(motion::CmdFixDepthConf msg, std::shared_ptr<const DepthRegulConfig> config);
    ~DepthRegulator();

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;
    double cmd_depth;
    double target_depth;
    CoordSystem coord_system;

    std::shared_ptr<const DepthRegulConfig> config;
};
