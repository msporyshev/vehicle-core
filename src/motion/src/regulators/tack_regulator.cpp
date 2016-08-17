#include "regulators/tack_regulator.h"

#include "registry.h"

#include <utils/math_u.h>
#include <utils/earth.h>

using namespace std;
using namespace utils;

using motion::CmdFixTack;

namespace {

Point2d gps_to_meter(navig::MsgGlobalPosition target, navig::MsgGlobalPosition origin)
{
    Point2d res;
    res.x = meter_deg_lat(target.latitude - origin.latitude, origin.latitude);
    res.y = meter_deg_long(target.longitude - origin.longitude, origin.latitude);

    return res;
}

} // namespace

TackRegulConfig::TackRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    YAML::Node dependencies = config.read_as<YAML::Node>("dependencies");
    heading_config = make_shared<HeadingRegulConfig>(YamlReader(dependencies["heading"]));
}

TackRegulator::TackRegulator(CmdFixTack msg, std::shared_ptr<const TackRegulConfig> config):
    Regulator(msg.id, {Axis::TX, Axis::MZ}, msg.timeout),
    cmd(msg),
    thrust(msg.thrust),
    config(config)
{

}

void TackRegulator::initialize(const NavigInfo& msg)
{
    tack = gps_to_meter(cmd.end, cmd.begin);
    current_pos = gps_to_meter(msg.position, cmd.begin);

    motion::CmdFixHeading fix_heading_msg;
    fix_heading_msg.value = kurs_point1_to_point2(current_pos.x, current_pos.y,
        tack.x, tack.y);
    fix_heading_msg.coord_system = static_cast<short>(CoordSystem::ABS);

    LOG << "initializing tack regulator" << endl;

    heading_regulator = make_shared<HeadingRegulator>(fix_heading_msg, config->heading_config);
    heading_regulator->activate();
    heading_regulator->handle_navig(msg);
    add_dependency(heading_regulator);
}

void TackRegulator::update(const NavigInfo& msg)
{
    current_pos = gps_to_meter(msg.position, cmd.begin);

    double a = tack.y;
    double b = -tack.x;
    double dist = (a * current_pos.x + b * current_pos.y) / norm(tack);

    double heading_delta = (-dist) * config->kp;
    double tack_heading = kurs_point1_to_point2(0, 0, tack.x, tack.y);

    heading_regulator->set_target_heading(tack_heading + heading_delta);

    double t = (tack.x * current_pos.x  + tack.y * current_pos.y) / norm(tack);

    set_success(t > norm(tack));

    set_thrusts({{Axis::TX, thrust}});
}

REG_REGUL(tack, TackRegulator, CmdFixTack, TackRegulConfig);
