#include "regulators/pitch_regulator.h"

#include "registry.h"
#include <utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixPitch;

PitchRegulConfig::PitchRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(accuracy);
    bound_vel = config.is_param_readable(max_finishing_vel, "max_finishing_vel");
}

PitchRegulator::PitchRegulator(CmdFixPitch msg, shared_ptr<const PitchRegulConfig> config):
    Regulator(msg.id, {Axis::MY}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    cmd_pitch(normalize_degree_angle(msg.value)),
    coord_system(static_cast<CoordSystem>(msg.coord_system)),
    config(config)
{

}

void PitchRegulator::initialize(const NavigInfo& msg)
{
    if (coord_system == CoordSystem::ABS) {
        target_pitch = cmd_pitch;
    } else {
        // Потенциальный баг
        target_pitch = normalize_degree_angle(cmd_pitch + msg.heading);
    }
    set_log_headers({"ptch_w", "ptch_r"});
}

void PitchRegulator::update(const NavigInfo& msg)
{
    double pitch = normalize_degree_angle(msg.pitch);
    double err = normalize_degree_angle(target_pitch - pitch);
    double err_d = -msg.pitch_rate;
    double thrust = controller.update(err, err_d);
    if (config->bound_vel) {
        set_success(fabs(err) < config->accuracy && fabs(msg.pitch_rate) < config->max_finishing_vel);
    } else {
        set_success(fabs(err) < config->accuracy);
    }
    set_thrusts({{Axis::MY, thrust}});
    set_log_values({target_pitch, pitch});
}

REG_REGUL(pitch, PitchRegulator, CmdFixPitch, PitchRegulConfig);
