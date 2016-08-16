#pragma once

#include "regulator.h"
#include "heading_regulator.h"

#include <motion/CmdFixTarget.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdReconfigure.h>

#include <point/point.h>
#include <move_mode.h>
#include <coord_system.h>

struct TargetRegulConfig: PidRegulConfig
{
    TargetRegulConfig(const YamlReader& config);

    std::shared_ptr<HeadingRegulConfig> heading_config;
};

class TargetRegulator : public Regulator
{
public:
    TargetRegulator(motion::CmdFixTarget msg, std::shared_ptr<const TargetRegulConfig> config);

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    std::shared_ptr<HeadingRegulator> heading_regulator;

    Point2d cmd_position;
    MoveMode mode;
    CoordSystem coord_system;

    std::shared_ptr<const TargetRegulConfig> config;
};
