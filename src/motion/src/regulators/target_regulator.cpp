#include "regulators/target_regulator.h"

#include "registry.h"

#include <utils/math_u.h>

using namespace std;
using namespace utils;

using motion::CmdFixTarget;

TargetRegulConfig::TargetRegulConfig(const YamlReader& config): PidRegulConfig(config)
{
    YAML::Node dependencies = config.read_as<YAML::Node>("dependencies");
    heading_config = make_shared<HeadingRegulConfig>(YamlReader(dependencies["heading"]));
}

TargetRegulator::TargetRegulator(CmdFixTarget msg, std::shared_ptr<const TargetRegulConfig> config):
    Regulator(msg.id, {Axis::MZ}, msg.timeout),
    cmd_position(Point2d(msg.x, msg.y)),
    config(config)
{

}

void TargetRegulator::initialize(const NavigInfo& msg)
{
    auto current_position = msg.position;

    motion::CmdFixHeading fix_heading_msg;
    fix_heading_msg.value = kurs_point1_to_point2(current_position.x, current_position.y,
        cmd_position.x, cmd_position.y);
    fix_heading_msg.coord_system = static_cast<short>(CoordSystem::ABS);

    LOG << "initializing target regulator" << endl;
    LOG << "msg.heading: " << msg.heading << endl;
    LOG << "target_heading: " << fix_heading_msg.value << endl;

    heading_regulator = make_shared<HeadingRegulator>(fix_heading_msg, config->heading_config);
    heading_regulator->activate();
    heading_regulator->handle_navig(msg);
    add_dependency(heading_regulator);
}

void TargetRegulator::update(const NavigInfo& msg)
{
    Point2d current_position = msg.position;

    double target_heading = kurs_point1_to_point2(current_position.x, current_position.y,
        cmd_position.x, cmd_position.y);

    heading_regulator->set_target_heading(target_heading);
    set_success(heading_regulator->has_succeeded());
}

REG_REGUL(target, TargetRegulator, CmdFixTarget, TargetRegulConfig);
