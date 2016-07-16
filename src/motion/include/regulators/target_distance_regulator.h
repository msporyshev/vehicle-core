#pragma once

#include "regulator.h"
#include "target_regulator.h"

#include <motion/CmdFixTargetDistance.h>
#include <motion/CmdFixTarget.h>
#include <motion/CmdReconfigure.h>

#include <coord_system.h>

#include <point/point.h>

struct TargetDistanceRegulConfig: PidRegulConfig
{
    TargetDistanceRegulConfig(const YamlReader& config);

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;

    std::shared_ptr<TargetRegulConfig> target_config;
};

class TargetDistanceRegulator : public Regulator
{
public:
    TargetDistanceRegulator(motion::CmdFixTargetDistance msg,
        std::shared_ptr<const TargetDistanceRegulConfig> config);

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController controller;

    std::shared_ptr<TargetRegulator> target_regulator;

    Point2d target_position;

    Point2d cmd_position;
    double cmd_distance;

    CoordSystem coord_system;

    std::shared_ptr<const TargetDistanceRegulConfig> config;
};
