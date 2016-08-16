#pragma once

#include "regulator.h"
#include "regul_config.h"

#include "controllers/pid_controller.h"

#include <motion/CmdFixHeading.h>
#include <coord_system.h>

struct HeadingRegulConfig: PidRegulConfig
{
    HeadingRegulConfig(const YamlReader& config);

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;
};

class HeadingRegulator : public Regulator
{
public:
    HeadingRegulator(motion::CmdFixHeading msg, std::shared_ptr<const HeadingRegulConfig> config);

    // Эта функция необходима для регуляторов с "плавающим" курсом
    void set_target_heading(double heading) { target_heading = heading; }
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
