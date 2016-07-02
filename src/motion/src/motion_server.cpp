#include "motion_server.h"

#include <iostream>
#include <iomanip>

#include "log.h"

#include <motion/MsgCmdStatus.h>
#include <tcu/CmdForce.h>

#include <array>
#include <functional>
#include <memory>
#include <stdexcept>

#include "regulators/regulator.h"

#include "registry.h"

#include <libauv/utils/math_u.h>

using namespace std;

const string MotionServer::NODE_NAME = "motion";

MotionServer::MotionServer(ipc::Communicator& communicator, const YamlReader& config) :
    communicator_(communicator)
{
    read_config(config);
}

MotionServer::~MotionServer()
{

}

void MotionServer::init_ipc()
{
    angles_msg_ = communicator_.subscribe<navig::MsgAngle>("navig");
    rates_msg_ = communicator_.subscribe<navig::MsgAngleRate>("navig");
    depth_msg_ = communicator_.subscribe<navig::MsgDepth>("navig");
    height_msg_ = communicator_.subscribe<navig::MsgHeight>("navig");
    position_msg_ = communicator_.subscribe<navig::MsgLocalPosition>("navig");
    velocity_msg_ = communicator_.subscribe<navig::MsgPlaneVelocity>("navig");

    cmd_status_pub_ = communicator_.advertise<motion::MsgCmdStatus>();
    regul_pub_ = communicator_.advertise<tcu::CmdForce>();
}

void MotionServer::handle_angles(const navig::MsgAngle& msg)
{
    navig.heading = msg.heading;
    navig.pitch = msg.pitch;
    navig.roll = msg.roll;
}

void MotionServer::handle_rate(const navig::MsgAngleRate& msg)
{
    navig.heading_rate = msg.heading;
    navig.pitch_rate = msg.pitch;
    navig.roll_rate = msg.roll;
}

void MotionServer::handle_depth(const navig::MsgDepth& depth)
{
    navig.depth = depth.distance;
    navig.velocity_depth = depth.velocity;
}

void MotionServer::handle_height(const navig::MsgHeight& height)
{
    navig.height = height.distance;
    navig.velocity_height = height.velocity;
}

void MotionServer::handle_position(const navig::MsgLocalPosition& pos)
{
    navig.position.x = pos.north;
    navig.position.y = pos.east;
}

void MotionServer::handle_velocity(const navig::MsgPlaneVelocity& velocity)
{
    navig.velocity_forward = velocity.forward;
    navig.velocity_right = velocity.right;

    // navig.velocity_north = velocity.north;
    // navig.velocity_east = velocity.east;
}

void MotionServer::run()
{
    ipc::EventLoop loop(freq_);
    while (loop.ok()) {
        read_msg(angles_msg_, &MotionServer::handle_angles);
        read_msg(rates_msg_, &MotionServer::handle_rate);
        read_msg(depth_msg_, &MotionServer::handle_depth);
        read_msg(height_msg_, &MotionServer::handle_height);
        read_msg(position_msg_, &MotionServer::handle_position);
        read_msg(velocity_msg_, &MotionServer::handle_velocity);
        process_navig(navig);
        update_activity_list();
    }
}

YAML::Node MotionServer::make_regul_config(string name, YamlReader config, vector<string> rejected_dependencies) const
{
    YAML::Node result;
    if (config.is_param_readable(result, name)) {
        result = config.read_as<YAML::Node>(name);
    } else {
        return result;
    }
    YamlReader regul_config = YamlReader(result).set_silent_mode();

    rejected_dependencies.push_back(name);
    vector<string> dependencies;
    if (regul_config.is_param_readable(dependencies, "dependencies")) {
        YAML::Node dependencies_node;
        for (auto dep : dependencies) {
            for (auto rej_dep : rejected_dependencies) {
                if (dep == rej_dep) {
                    throw std::runtime_error("cycled config dependencies are not allowed");
                }
            }
            auto dep_node = make_regul_config(dep, config, rejected_dependencies);
            dependencies_node[dep] = dep_node;
        }
        result["dependencies"] = dependencies_node;
    }
    return result;
}

void MotionServer::read_config(YamlReader config)
{
    config.read_param(freq_, "frequency");
    config.read_param(timeout_silence_, "timeout_silence");
    config.read_param(timeout_not_respond_, "timeout_not_respond");

    vector<string> regul_names;
    config.read_param(regul_names, "regulators");

    YamlReader params = YamlReader(config.read_as<YAML::Node>("params")).set_silent_mode();
    config.set_silent_mode();
    for (auto name : regul_names) {
        auto producers = Registry::get(name);
        for (auto p : producers) {
            p->init(shared_ptr<RegulStorage>(&pending_list),
                YamlReader(make_regul_config(name, params)).set_silent_mode(),
                communicator_);
        }
        LOG << "regul producer " << name << " initialized" << endl;
    }

    auto limits_config = YamlReader(config.read_as<YAML::Node>("limits"));
    LOG << "reading thrust limits" << endl;
    limits_config.read_param(forward_limit, "forward");
    limits_config.read_param(right_limit, "right");
    limits_config.read_param(down_limit, "down");
    limits_config.read_param(mforward_limit, "mforward");
    limits_config.read_param(mright_limit, "mright");
    limits_config.read_param(mdown_limit, "mdown");
}

void MotionServer::update_activity_list()
{
    vector<shared_ptr<Regulator>> active_reguls = active_list.get_stored();
    vector<shared_ptr<Regulator>> new_active_reguls;
    for (auto r : active_reguls) {
        if (r->is_actual()) {
            if (r->has_succeeded()) {
                publish_cmd_status(r->get_id(), CmdStatus::SUCCESS);
            }
            new_active_reguls.push_back(r);
        } else {
            r->finish();
            publish_cmd_status(r->get_id(), CmdStatus::TIMEOUT);
        }
    }
    active_list.clear_stored();
    active_list.add(new_active_reguls);

    for (auto r : pending_list.get_conflicted()) {
        r->finish();
        publish_cmd_status(r->get_id(), CmdStatus::STOPPED);
    }
    for (auto r : pending_list.get_stored()) {
        r->activate();
    }
    active_list.add(pending_list.get_stored());
    for (auto r : active_list.get_conflicted()) {
        r->finish();
        publish_cmd_status(r->get_id(), CmdStatus::STOPPED);
    }

    pending_list.clear_all();
    active_list.clear_conflicted();
}

void MotionServer::update_thrusts(const NavigInfo& msg)
{
    tcu::CmdForce result;

    array<double, 6> thrusts = { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };

    string log_str = "";
    for (auto r : active_list.get_stored()) {
        r->handle_navig(msg);
        auto cur_thrusts = r->get_thrusts();
        auto log_headers = r->get_log_headers();
        auto log_values = r->get_log_values();
        for (size_t i = 0; i < log_headers.size(); i++) {
            log_str += log_headers[i] + ": ";
            log_str += to_string(log_values[i]) + "\t";
        }
        for (auto t : cur_thrusts) {
            thrusts[static_cast<int>(t.axis)] = t.value;
        }
    }


    result.forward = bound(thrusts[static_cast<int>(Axis::TX)], forward_limit);
    result.right = bound(thrusts[static_cast<int>(Axis::TY)], right_limit);
    result.down = bound(thrusts[static_cast<int>(Axis::TZ)], down_limit);
    result.mforward = bound(thrusts[static_cast<int>(Axis::MX)], mforward_limit);
    result.mright = bound(thrusts[static_cast<int>(Axis::MY)], mright_limit);
    result.mdown = bound(thrusts[static_cast<int>(Axis::MZ)], mdown_limit);

    LOG << "forward: " << result.forward << "\t"
        << "right: " << result.right << "\t"
        << "down: " << result.down << "\t"
        << "mforward: " << result.mforward << "\t"
        << "mright: " << result.mright << "\t"
        << "mdown: " << result.mdown << "\t"
        << log_str << endl;
    regul_pub_.publish(result);
}

void MotionServer::publish_cmd_status(int id, CmdStatus status)
{
    motion::MsgCmdStatus msg;
    msg.id = id;
    msg.status = static_cast<char>(status);

    string status_str;
    switch(status) {
        case CmdStatus::SUCCESS:
            status_str = "SUCCESS";
        break;
        case CmdStatus::PROCESSING:
            status_str = "PROCESSING";
        break;
        case CmdStatus::TIMEOUT:
            status_str = "TIMEOUT";
        break;
        case CmdStatus::STOPPED:
            status_str = "STOPPED";
        break;
    }
    LOG << "command #" << id << ": " << status_str << endl;

    cmd_status_pub_.publish(msg);
}

void MotionServer::process_navig(const NavigInfo& msg)
{
    update_thrusts(msg);
}

double MotionServer::bound(double num, double limit)
{
    double abs_limit = fabs(limit);
    if (num < -abs_limit) {
        num = -abs_limit;
    }
    if (num > abs_limit) {
        num = abs_limit;
    }
    return num;
}

tcu::CmdForce MotionServer::convert(const tcu::CmdForce& msg) const
{
    tcu::CmdForce result;
    result.forward = msg.right;
    result.right = msg.forward;
    result.down = -msg.down;
    result.mforward = -msg.mright;
    result.mright = msg.mforward;
    result.mdown = msg.mdown;

    return result;
}
