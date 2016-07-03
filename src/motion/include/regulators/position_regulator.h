#pragma once

#include "regulator.h"
#include "heading_regulator.h"

#include <motion/CmdFixPosition.h>
#include <motion/CmdFixPositionConf.h>
#include <motion/CmdFixHeading.h>

#include <libauv/point/point.h>
#include <move_mode.h>
#include <coord_system.h>

struct PositionRegulConfig
{
    PositionRegulConfig(const YamlReader& config);
    ~PositionRegulConfig();

    double cruiser_dist;

    double fwd_kp;
    double fwd_ki;
    double fwd_kd;

    double side_kp;
    double side_ki;
    double side_kd;

    double accuracy;
    double max_finishing_vel;
    bool bound_vel;

    std::shared_ptr<HeadingRegulConfig> heading_config;
};

class PositionRegulator : public Regulator
{
public:
    PositionRegulator(motion::CmdFixPosition msg, std::shared_ptr<const PositionRegulConfig> config);
    PositionRegulator(motion::CmdFixPositionConf msg, std::shared_ptr<const PositionRegulConfig> config);
    ~PositionRegulator();

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController fwd_controller;
    PIDController side_controller;

    std::shared_ptr<HeadingRegulator> heading_regulator;

    libauv::Point2d target_position;

    libauv::Point2d cmd_position;
    MoveMode mode;
    CoordSystem coord_system;

    std::shared_ptr<const PositionRegulConfig> config;

    libauv::Point2d get_current_position(const NavigInfo& msg);
};
