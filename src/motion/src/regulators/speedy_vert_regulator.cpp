#include "regulators/speedy_vert_regulator.h"

#include "registry.h"

using namespace std;

using motion::CmdFixVert;

SpeedyVertRegulConfig::SpeedyVertRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    config.SET_PARAM(min_height);

    config.SET_PARAM(accuracy);
    config.SET_PARAM(max_delta_depth);
}

SpeedyVertRegulator::SpeedyVertRegulator(CmdFixVert msg, shared_ptr<const SpeedyVertRegulConfig> config):
    Regulator(msg.id, {Axis::MY}, msg.timeout),
    controller(config->kp, config->ki, config->kd),
    mode(static_cast<SpeedyVertMode>(msg.mode)),
    config(config)
{
    if (mode == SpeedyVertMode::DEPTH) {
        set_log_headers({"dpth_w", "dpth_r"});
    } else {
        if (target_value < config->min_height) {
            target_value = config->min_height;
        }
        set_log_headers({"hght_w", "hght_r"});
    }
}

SpeedyVertRegulator::~SpeedyVertRegulator()
{

}

void SpeedyVertRegulator::update(const NavigInfo& msg)
{
    double value = mode == SpeedyVertMode::DEPTH ? msg.depth : msg.height;
    double err = target_value - value;
    if (err < -config->max_delta_depth) err = -config->max_delta_depth;
    if (err > config->max_delta_depth) err = config->max_delta_depth;
    double thrust = 0.0;

    // Пока используется только обычный ПИД-регулятор
    if (mode == SpeedyVertMode::DEPTH) {
        thrust = controller.update(err, -msg.velocity_depth);
    } else {
        thrust = controller.update(err, -msg.velocity_height);
    }
    set_thrusts({{Axis::MY, thrust}});
    set_success(err, config->accuracy);
    set_log_values({target_value, value});
}

REG_REGUL(speedy_vert, SpeedyVertRegulator, CmdFixVert, SpeedyVertRegulConfig);
