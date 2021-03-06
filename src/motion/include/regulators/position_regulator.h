#pragma once

#include "regulator.h"
#include "heading_regulator.h"

#include <motion/CmdFixPosition.h>
#include <motion/CmdFixHeading.h>
#include <motion/CmdReconfigure.h>

#include <point/point.h>
#include <move_mode.h>
#include <coord_system.h>

struct PositionRegulConfig: RegulConfig
{
    PositionRegulConfig(const YamlReader& config);

    void reconfigure(const motion::CmdReconfigure& msg) {
        if (msg.name == "fwd_kp") {
            fwd_kp = msg.value;
        } else if (msg.name == "fwd_kd") {
            fwd_kd = msg.value;
        } else if (msg.name == "fwd_ki") {
            fwd_ki = msg.value;
        } else if (msg.name == "side_kp") {
            side_kp = msg.value;
        } else if (msg.name == "side_kd") {
            side_kd = msg.value;
        } else if (msg.name == "side_ki") {
            side_ki = msg.value;
        }
    }

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

protected:
    virtual void initialize(const NavigInfo& msg) override;
    virtual void update(const NavigInfo& msg) override;

private:
    PIDController fwd_controller;
    PIDController side_controller;

    std::shared_ptr<HeadingRegulator> heading_regulator;

    Point2d target_position;

    Point2d cmd_position;
    MoveMode mode;
    CoordSystem coord_system;

    std::shared_ptr<const PositionRegulConfig> config;

    Point2d get_current_position(const NavigInfo& msg);
};
