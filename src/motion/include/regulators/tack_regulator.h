#pragma once

#include "regulator.h"
#include "heading_regulator.h"
#include "position_regulator.h"

#include <motion/CmdFixTack.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdReconfigure.h>

#include <point/point.h>
#include <move_mode.h>
#include <coord_system.h>

struct TackRegulConfig: PidRegulConfig
{
    TackRegulConfig(const YamlReader& config);

    std::shared_ptr<HeadingRegulConfig> heading_config;
};

class TackRegulator : public Regulator
{
public:
    TackRegulator(motion::CmdFixTack msg, std::shared_ptr<const TackRegulConfig> config);

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    std::shared_ptr<HeadingRegulator> heading_regulator;
    motion::CmdFixTack cmd;

    Point2d tack;
    Point2d current_pos;
    double thrust;

    CoordSystem coord_system;
    std::shared_ptr<const TackRegulConfig> config;
};
